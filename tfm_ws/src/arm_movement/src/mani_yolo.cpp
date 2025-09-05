// Importacion de librerias y mensajes personalizados
#include <memory>
#include <chrono>
#include <atomic>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <yolo_msgs/msg/detection_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;


// Variables globales para controlar el estado del sistema:
// - task_done: indica si la tarea se ha completado y se puede finalizar el nodo.
// - head_tilted: indica si la cabeza ha sido inclinada hacia abajo.
// - detection_timer_started y last_detection_time: para controlar el tiempo sin detecciones.
// - second_timer_started y second_detection_time: para controlar el tiempo despues de inclinar la cabeza.
std::atomic_bool task_done(false);
std::atomic_bool head_tilted(false);
rclcpp::Time last_detection_time;
bool detection_timer_started = false;

bool second_timer_started = false;
rclcpp::Time second_detection_time;

// Estructura para almacenar la ultima posicion detectada y el tiempo asociado.
// Se utiliza para comprobar si la deteccion es estable durante al menos 5 segundos.
struct DetectionTracker {
    geometry_msgs::msg::Point last_position;
    rclcpp::Time last_time;
    bool initialized = false;
};

// Informacion sobre la forma y dimensiones del objeto detectado.
// Se utiliza para construir la colision en la escena de MoveIt.
struct ShapeInfo {
  uint8_t primitive_type;
  std::vector<double> dimensions;
};

// Offsets que se aplican sobre la posicion del objetp:
// - approach: cuando se aproxima al objeto.
// - grab: para la posicion exacta del agarre.
struct OffsetsManipulation {
    double x_approach;
    double y_approach;
    double z_approach;
    double x_grab;
    double y_grab;
    double z_grab;
};

// Mapeo de clases soportadas a su tipo de primitiva (BOX o CYLINDER) y dimensiones.
static const std::unordered_map<std::string, ShapeInfo> kObjectShapes = {
  {"bottle",      {shape_msgs::msg::SolidPrimitive::CYLINDER,      {0.122, 0.033}}},
  {"can",         {shape_msgs::msg::SolidPrimitive::CYLINDER,      {0.122, 0.033}}},
  {"book_side",   {shape_msgs::msg::SolidPrimitive::BOX,           {0.057, 0.125, 0.182}}},
  {"book_front",  {shape_msgs::msg::SolidPrimitive::BOX,           {0.125, 0.057, 0.182}}},
  {"cube",        {shape_msgs::msg::SolidPrimitive::BOX,           {0.055, 0.055, 0.055}}},
  {"bag",         {shape_msgs::msg::SolidPrimitive::CYLINDER,      {0.23, 0.07}}}
};


// Funcion que controla la apertura o cierre de la pinza del robot.
// Establece los valores objetivo para las articulaciones de la pinza y ejecuta el movimiento usando MoveIt.
// 
// - logger: objeto para imprimir logs por consola.
// - gripper_group: interfaz del grupo de movimiento de la pinza.
// - close: si es true, cierra la pinza; si es false, la abre.
// - close_value: valor al que cerrar (por defecto: 0.013).
// - open_value: valor al que abrir (por defecto: 0.043).
//  
// - salida: true si el movimiento se ejecuto correctamente, false si hay algun fallo.
bool control_gripper(const rclcpp::Logger &logger,
                     moveit::planning_interface::MoveGroupInterface &gripper_group, bool close,
                     const double close_value = 0.013,
                     const double open_value = 0.043)
{
    std::map<std::string, double> target_positions;
    if (close) {
        target_positions["gripper_left_finger_joint"] = close_value;
        target_positions["gripper_right_finger_joint"] = close_value;
    } else {
        target_positions["gripper_left_finger_joint"] = open_value;
        target_positions["gripper_right_finger_joint"] = open_value;
    }

    gripper_group.setJointValueTarget(target_positions);
    bool success = static_cast<bool>(gripper_group.move());
    if (success) {
        RCLCPP_INFO(logger, "Gripper %s successfully", close ? "closed" : "opened");
        return true;
    } else {
        RCLCPP_ERROR(logger, "Failed to move gripper");
    }

    return false;
}


// Funcion que crea e inserta un objeto de colision (una mesa) en la escena de planificacion de MoveIt.
// La mesa es donde se situan los objeros a manipular.
//
// - arm_group: interfaz del grupo de movimiento del brazo, desde el cual se obtiene el frame de referencia.
//
// - salida: interfaz de escena de planificacion (PlanningSceneInterface) con la mesa ya anadida.
moveit::planning_interface::PlanningSceneInterface  create_scene(moveit::planning_interface::MoveGroupInterface &arm_group)
{
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    moveit_msgs::msg::CollisionObject table;
    table.header.frame_id = arm_group.getPlanningFrame();
    table.id = "table";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {0.75, 1.82, 0.81};

    geometry_msgs::msg::Pose table_pose;
    table_pose.position.x = 1;
    table_pose.position.y = 0.0;
    table_pose.position.z = 0.41;

    table.primitives.push_back(primitive);
    table.primitive_poses.push_back(table_pose);
    table.operation = table.ADD;

    planning_scene_interface.applyCollisionObject(table);

    return planning_scene_interface;
}


// Funcion que publica una trayectoria para inclinar la cabeza del robot hacia abajo o volver a la posicion inicial.
// Se utiliza para mejorar la visibilidad del robot cuando no se detectan objetos durante un tiempo.
//
// - node: puntero al nodo ROS 2 desde donde se publica la trayectoria.
// - direction: 'd' para inclinar hacia abajo (default), cualquier otro valor para volver a la posicion original.
void tilt_head(std::shared_ptr<rclcpp::Node> node, char direction = 'd')
{
    auto pub = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/head_controller/joint_trajectory", 10);

    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = {"head_1_joint", "head_2_joint"};

    trajectory_msgs::msg::JointTrajectoryPoint point;
    
    float tilt_angle = (direction == 'd') ? -0.35 : -0.03;
    point.positions = {0.0, tilt_angle};
    point.time_from_start = rclcpp::Duration::from_seconds(1.0);

    traj.points.push_back(point);

    rclcpp::sleep_for(500ms);
    pub->publish(traj);

    RCLCPP_INFO(node->get_logger(), "Published head joint trajectory");

    rclcpp::sleep_for(3s);
}


// Funcion que planifica y ejecuta el movimiento del brazo hasta una posicion especifica.
// Permite aplicar offsets y establecer diferentes orientaciones del efector final segun el tipo de agarre.
//
// - logger: objeto para imprimir logs por consola.
// - arm_group: interfaz del grupo de movimiento del brazo.
// - x, y, z: coordenadas objetivo (sin offsets).
// - x_offset, y_offset, z_offset: desplazamientos adicionales aplicados a la posicion objetivo.
// - approach_type: tipo de aproximacion que define la orientacion del efector.
// - set_orientation: si es true, se asigna la orientacion correspondiente al tipo de aproximacion.
//
// - salida: true si la planificacion y ejecucion se realizaron con exito; false si fallo.
bool move_arm(const rclcpp::Logger &logger, 
              moveit::planning_interface::MoveGroupInterface &arm_group,
              double x,
              double y,
              double z,
              double const x_offset = 0.0,
              double const y_offset = 0.0,
              double const z_offset = 0.0,
              const std::string &approach_type = "front",
              bool set_orientation = false)
{
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x - x_offset;
    target_pose.position.y = y - y_offset;
    target_pose.position.z = z + z_offset;

    tf2::Quaternion q;

    if (approach_type == "above") {
        q = tf2::Quaternion (0.5, 0.5, -0.5, 0.5);
    } 
    else if (approach_type == "above_front") {
        q = tf2::Quaternion (0.71244, 0.0, 0.0, 0.70127);
    } 
    else if (approach_type == "side_der") {
        q.setRPY(0.0, 0.0, M_PI/2.0);
    } 
    else if (approach_type == "side_izq")
    {
        q.setRPY(0.0, 0.0, -M_PI/2.0);
    }
    else {
        q = tf2::Quaternion (0.0, 0.0, 0.0, 1);
    }

    if (set_orientation)
    {
        target_pose.orientation = tf2::toMsg(q);
    }

    arm_group.setStartStateToCurrentState();
    arm_group.setPoseTarget(target_pose);

    RCLCPP_INFO(logger, "Target (%.3f, %.3f, %.3f)", x, y, z);
    RCLCPP_INFO(logger, "Target (%.3f, %.3f, %.3f)", target_pose.position.x, target_pose.position.y, target_pose.position.z);
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(arm_group.plan(plan));
    if (success && !plan.trajectory.joint_trajectory.points.empty()) {
        arm_group.execute(plan);
        return true;
    } 
    else {
        RCLCPP_ERROR(logger, "Planning failed for target (%.3f, %.3f, %.3f)", target_pose.position.x, target_pose.position.y, target_pose.position.z);
    }

    return false;
}


// Funcion que devuelve la altura del objeto segun su clase.
// La altura se obtiene a partir de las dimensiones definidas en kObjectShapes.
// Se usa para ajustar correctamente la posicion Z en agarres y colocacion.
//
// - object_class: clase del objeto detectado.
//
// - salida: altura del objeto. Si la clase no se encuentra, devuelve un valor por defecto de 0.03.
double get_object_height(const std::string &object_class) 
{
    auto it = kObjectShapes.find(object_class);
    if (it == kObjectShapes.end()) return 0.03;

    const auto &shape = it->second;
    if (shape.primitive_type == shape_msgs::msg::SolidPrimitive::CYLINDER) {
        return shape.dimensions[0];
    } else {
        return shape.dimensions[2];
    }
}

// Funcion que devuelve los offsets de aproximacion y agarre para un tipo de agarre.
//
// - approach_type: tipo de aproximacion.
//
// - salida: estructura OffsetsManipulation con los valores correspondientes para approach y grab.
OffsetsManipulation get_offsets_for_approach(const std::string &approach_type)
{
    OffsetsManipulation offsets = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    if (approach_type == "above") {
        offsets.z_approach = 0.12;
        offsets.z_grab = 0.085;
    }
    else if (approach_type == "side_der") {
        offsets.y_approach = 0.20;
        offsets.y_grab = 0.06;
    }
    else if (approach_type == "side_izq") {
        offsets.y_approach = -0.20;
        offsets.y_grab = -0.06;
    }
    else if (approach_type == "above_front") {
        offsets.x_approach = 0.12;
        offsets.z_approach = 0.274;

        offsets.x_grab = 0.055;
        offsets.z_grab = 0.185;
    }
    else {
        offsets.x_approach = 0.20;
        offsets.x_grab = 0.06;
    }

    return offsets;
}


// Funcion que ejecuta el proceso de agarre del objeto detectado, incluyendo la aproximacion, apertura de la pinza,
// movimiento final de agarre, union del objeto a la pinza y cierre.
//
// - logger: objeto para imprimir logs por consola.
// - arm_group: interfaz del grupo de movimiento del brazo.
// - planning_scene_interface: interfaz de la escena de planificacion.
// - object_class: clase del objeto detectado.
// - gripper_group: interfaz del grupo de movimiento de la pinza.
// - x, y, z: posicion del objeto detectado.
// - approach_type: tipo de aproximacion.
// - offsets: estructura con los offsets de aproximacion y agarre.
//
// - salida: objeto AttachedCollisionObject que representa el objeto agarrado. Si falla el agarre, devuelve objeto vacio.
moveit_msgs::msg::AttachedCollisionObject grab_object(const rclcpp::Logger &logger,
                                                      moveit::planning_interface::MoveGroupInterface &arm_group,
                                                      moveit::planning_interface::PlanningSceneInterface &planning_scene_interface,
                                                      const std::string object_class,
                                                      moveit::planning_interface::MoveGroupInterface &gripper_group,
                                                      double x,
                                                      double y,
                                                      double z,
                                                      const std::string &approach_type,
                                                      const OffsetsManipulation &offsets)
{
    double open_value_gripper = 0.043;
    double close_value_gripper = 0.013;

    // Si la aproximacion no es desde arriba, primero se mueve a una posicion de aproximacion
    if (approach_type != "above"){
        RCLCPP_INFO(logger, "Aproching to object");
        move_arm(logger, arm_group, x, y, z, offsets.x_approach, offsets.y_approach, offsets.z_approach, approach_type, true);
    }

    // Ajuste de valores personalizados de apertura/cierre para cuando el objeto es una bolsa, ya que el espacio entre el asa y el cuerpo es estrecho
    if (approach_type == "above_front"){
        open_value_gripper = 0.022;
        close_value_gripper = 0.01;
    }

    // Abrir pinza antes de hacer el agarre final
    control_gripper(logger, gripper_group, false, 0.0, open_value_gripper);
    rclcpp::sleep_for(1s);

    // Calculo de la altura real del objeto y definicion de la altura del punto de agarre
    double obj_height = get_object_height(object_class);
    double z_grasp = 0.82 + obj_height / 2.0;

    // Movimiento final para posicionarse y agarrar el objeto
    bool done = move_arm(logger, arm_group, x, y, z_grasp, offsets.x_grab, offsets.y_grab, offsets.z_grab, approach_type, true);
    // Si falla, se finaliza el nodo
    if (!done){
        rclcpp::shutdown();
        task_done = true;
        return {};
    }
    
    rclcpp::sleep_for(1s);

    // Crear el objeto de colision adjunto a la pinza
    moveit_msgs::msg::AttachedCollisionObject attached_object;
    attached_object.link_name = "gripper_grasping_frame";
    attached_object.object.header.frame_id = "gripper_grasping_frame";
    attached_object.object.id = (object_class == "book_front" || object_class == "book_side") ? "detected_book" :"detected_" + object_class ;

    attached_object.object.operation = attached_object.object.ADD;
    attached_object.touch_links = {"gripper_left_finger_link", "gripper_right_finger_link"};

    // Actualizar el objeto en la escena de planificacion
    planning_scene_interface.applyAttachedCollisionObject(attached_object);
    RCLCPP_INFO(logger, "Object attached to gripper");
    rclcpp::sleep_for(1s);

    // Cerrar la pinza para completar el agarre
    control_gripper(logger, gripper_group, true, close_value_gripper, 0.0);
    rclcpp::sleep_for(1s);

    return attached_object;
}


// Funcion que anade un objeto detectado a la escena de planificacion de MoveIt como CollisionObject.
// Usa la clase del objeto para determinar su forma y dimensiones, y lo posiciona en las coordenadas dadas.
//
// - logger: objeto para imprimir logs por consola.
// - planning_scene_interface: interfaz de la escena de planificacion.
// - frame_id: frame de referencia.
// - object_class: clase del objeto detectado.
// - x, y, z: posicion 3D del objeto.
void add_detected_object_to_scene(const rclcpp::Logger &logger,
                                  moveit::planning_interface::PlanningSceneInterface &planning_scene_interface,
                                  const std::string &frame_id,
                                  const std::string &object_class,
                                  double x,
                                  double y,
                                  double z)
{
    // Buscar las dimensiones y tipo de primitiva del objeto en kObjectShapes
    auto iter = kObjectShapes.find(object_class);
    if (iter == kObjectShapes.end()) return;
    const auto &object_shape_info = iter->second;

    // Crear el CollisionObject con su tipo y dimensiones
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = frame_id;
    object.id = (object_class == "book_front" || object_class == "book_side") ? "detected_book" :"detected_" + object_class ;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = object_shape_info.primitive_type;
    if (primitive.type == shape_msgs::msg::SolidPrimitive::CYLINDER) {
        primitive.dimensions = {
            object_shape_info.dimensions[0],
            object_shape_info.dimensions[1]
        };
    } 
    else {
        primitive.dimensions = {
            object_shape_info.dimensions[0],
            object_shape_info.dimensions[1],
            object_shape_info.dimensions[2]
        };
    }

    // Definir la pose del objeto segun su altura y clase
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    if (object_class == "bag"){ 
        pose.position.z = 0.83 + get_object_height(object_class) / 2.0;
    }
    else
    {
        pose.position.z = 0.82 + get_object_height(object_class) / 2.0;
    }
    
    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(pose);

    // Si el objeto es una bolsa, anadir asas como cajas adicionales
    if (object_class == "bag") {
        // Dimensiones y poses de asas:
        struct Handle { std::array<double,3> dim; geometry_msgs::msg::Pose pose; };
        std::vector<Handle> handles;

        // Asa central
        {
            Handle h;
            h.dim = {0.02, 0.14, 0.01};
            h.pose = pose;
            h.pose.position.x = x;
            h.pose.position.z = z + 0.28;
            handles.push_back(h);
        }
        // Asa lado 1
        {
            Handle h;
            h.dim = {0.02, 0.02, 0.05};
            h.pose = pose;
            h.pose.position.x = x;
            h.pose.position.y = y + 0.06;
            h.pose.position.z = z + 0.24;
            handles.push_back(h);
        }
        // Asa lado 2
        {
            Handle h;
            h.dim = {0.02, 0.02, 0.05};
            h.pose = pose;
            h.pose.position.x = x;
            h.pose.position.y = y - 0.06;
            h.pose.position.z = z + 0.24;
            handles.push_back(h);
        }

        // Anadir cada asa de una en una como una primitiva 
        for (const auto &h : handles) {
            shape_msgs::msg::SolidPrimitive box;
            box.type = shape_msgs::msg::SolidPrimitive::BOX;
            box.dimensions = { h.dim[0], h.dim[1], h.dim[2] };
            object.primitives.push_back(box);
            object.primitive_poses.push_back(h.pose);
        }
    }

    // Agregar el objeto completo a la escena de planificacion
    object.operation = object.ADD;
    planning_scene_interface.applyCollisionObject(object);
    RCLCPP_INFO(logger, "Added %s to the planning scene at (%.2f, %.2f, %.2f)", object_class.c_str(), x, y, z);
}


// Funcion que libera el objeto agarrado, lo elimina como AttachedCollisionObject y lo vuelve a anadir como CollisionObject.
// Despues de soltarlo, retira el brazo.
//
// - logger: objeto para imprimir logs por consola.
// - arm_group: interfaz del grupo de movimiento del brazo.
// - planning_scene_interface: interfaz de la escena de planificacion.
// - object_class: clase del objeto a soltar.
// - gripper_group: interfaz del grupo de movimiento de la pinza.
// - attached_object: objeto actualmente agarrado, sera eliminado de la pinza.
// - x, y, z: coordenadas donde se deja el objeto.
void detach_object(const rclcpp::Logger &logger,
                   moveit::planning_interface::MoveGroupInterface &arm_group,
                   moveit::planning_interface::PlanningSceneInterface &planning_scene_interface,
                   std::string object_class, 
                   moveit::planning_interface::MoveGroupInterface &gripper_group,
                   moveit_msgs::msg::AttachedCollisionObject &attached_object,
                   double x,
                   double y,
                   double z)
{
    double open_value_gripper = 0.043;

    // Abrir la pinza para soltar el objeto
    control_gripper(logger, gripper_group, false, 0.0, open_value_gripper);

    // Eliminar el objeto adjunto de la escena de planificacion
    attached_object.object.operation = attached_object.object.REMOVE;
    planning_scene_interface.applyAttachedCollisionObject(attached_object);
    RCLCPP_INFO(logger, "Detached object from gripper");

    rclcpp::sleep_for(1s);
    
    // Decidir si el libro debe ser book_side o book_front segun donde se suelta
    if (object_class == "book_front" || object_class == "book_side"){
        if ((y > 0.24 || y < -0.24)  && x < 0.85){
            object_class = "book_side";
        }
        else{
            object_class = "book_front";
        }       
    }

    // Volver a anadir el objeto como CollisionObject en la escena de planificacion
    add_detected_object_to_scene(logger, planning_scene_interface, arm_group.getPlanningFrame(), object_class, x, y, z);

    rclcpp::sleep_for(1s);

    // Movimiento para retirar el brazo. En el caso del cubo, se retira dejando la pinza sobre el cubo a 0.20
    if (object_class == "cube") {
        move_arm(logger, arm_group, x, 0.0, z, 0.0, 0.0, 0.20, "front", true);
        rclcpp::sleep_for(1s);
        control_gripper(logger, gripper_group, true, 0.0);
    }
    
    // En e resto de casos se retira el brazo a una posicion en frente del robot, lejos de la mesa
    else{
        RCLCPP_INFO(logger, "Moving to inital pose");
        move_arm(logger, arm_group, 0.42, 0.15, 0.80, 0.0, 0.0, 0.0, "side_izq", true);

        rclcpp::sleep_for(1s);
        control_gripper(logger, gripper_group, true, 0.0);
    }
}


// Funcion principal que ejecuta la manipulacion completa de un objeto: deteccion, agarre, solicitud de posicion deseada y colocacion final.
// Controla tanto el movimiento del brazo como de la pinza, y adapta la estrategia segun la clase y posicion del objeto.
//
// - node: puntero al nodo ROS 2.
// - arm_group: interfaz del grupo de movimiento del brazo.
// - gripper_group: interfaz del grupo de movimiento de la pinza.
// - planning_scene_interface: interfaz de la escena de planificacion..
// - object_class: clase del objeto detectado.
// - x, y, z: posicion actual del objeto detectado.
void manipulate_object(const std::shared_ptr<rclcpp::Node> &node,
                       moveit::planning_interface::MoveGroupInterface &arm_group,
                       moveit::planning_interface::MoveGroupInterface &gripper_group,
                       moveit::planning_interface::PlanningSceneInterface &planning_scene_interface,
                       const std::string &object_class,
                       double x,
                       double y,
                       double z)
{
    std::string approach_type, detach_type;
    // Elegir tipo de aproximacion y tipo de agarre segun la clase y posicion inicial del objeto
    if (object_class == "book_side") {
        if (y > 0.0)
        {
            approach_type = "side_der";
        }
        else
        {
            approach_type = "side_izq";
        }
        detach_type = "front";
    } 
    else if (object_class == "book_front") {
        approach_type = detach_type = "front";
    }
    else if (object_class == "cube") {
        approach_type = detach_type = "above";
    }
    else if (object_class == "bag")
    {
        approach_type = detach_type = "above_front";
    }
    else {
        approach_type = detach_type = "front";
    }

    // Anadir el objeto como ObjectCollision inicial antes de intentar agarrarlo
    add_detected_object_to_scene(node->get_logger(), planning_scene_interface, arm_group.getPlanningFrame(), object_class, x, y, z);
    rclcpp::sleep_for(1s);

    // Obtener los offsets adecuados segun el tipo de agarre
    OffsetsManipulation offsets = get_offsets_for_approach(approach_type);

    // Realizar el agarre del objeto y enlazarlo a la pinza
    auto attached_object = grab_object(node->get_logger(), arm_group, planning_scene_interface, object_class, gripper_group, x, y, z, approach_type, offsets);
    rclcpp::sleep_for(1s);
    
    // Verificar si ROS sigue activo antes de continuar
    if (!rclcpp::ok()) return;

    // Solicitar al usuario la posicion objetivo donde soltar el objeto
    double drop_x, drop_y;
    std::cout << "Introduce target position (x y): ";
    std::cin >> drop_x >> drop_y;

    // En el caos de que sea un libro: decidir si debe dejarse lateralmente o frontalmente segun el punto final
    if (object_class == "book_front" || object_class == "book_side"){
        if (drop_x < 0.85){
            if (drop_y > 0.24){
                detach_type = "side_der";
            }
            
            else if (drop_y < -0.24){
                detach_type = "side_izq";
            }            
        }   
    }

    // Calcular los offsets segun el tipo de detach y altura del objeto
    OffsetsManipulation offsets_detach = get_offsets_for_approach(detach_type);
    double obj_height = get_object_height(object_class);
    double drop_z = 0.82 + obj_height / 2.0;
    offsets_detach.z_grab = offsets_detach.z_grab+0.006;

    // Ajustes adicionales si el objeto es una bolsa, para dejarla a una altura segura y que el brazo pueda retirarse al abrir la pinza
    if (approach_type == "above_front"){
        drop_z = z;
        offsets_detach.z_grab = offsets.z_grab + 0.11;
        offsets_detach.x_grab = offsets_detach.x_grab + 0.04;
    }

    // Intentar mover el brazo a la posicion de desseada, repitiendo si es inalcanzable
    RCLCPP_INFO(node->get_logger(), "Planning for target (%.3f, %.3f, %.3f)", drop_x, drop_y, drop_z);
    RCLCPP_INFO(node->get_logger(), "Planning for target (%.3f, %.3f, %.3f)", drop_x+offsets_detach.x_grab, drop_y+offsets_detach.y_grab, drop_z+offsets_detach.z_grab);
    while (rclcpp::ok() && !move_arm(node->get_logger(), arm_group, drop_x, drop_y, drop_z, offsets_detach.x_grab, offsets_detach.y_grab, offsets_detach.z_grab, detach_type, true)) 
    {        
        std::cout << "Position unreachable. Try a new one (x y): ";
        std::cin >> drop_x >> drop_y;
    }
    
    // Ejecutar la liberacion del objeto y retirar el brazo
    rclcpp::sleep_for(1s);
    detach_object(node->get_logger(), arm_group, planning_scene_interface, object_class, gripper_group, attached_object, drop_x, drop_y, z);

    // Volver a inclinar la cabeza hasta la posicion inicial neutra
    tilt_head(node, 'u');
    task_done = true;
}


// Funcion callback que se ejecuta cada vez que se recibe un mensaje de deteccion 3D desde YOLO.
// Comprueba si hay objetos validos detectados, y si alguna deteccion es estable durante 5 segundos, lanza el proceso de manipulacion.
// Si no hay detecciones durante 5 segundos primero, inclina la cabeza hacia abajo, y si despues de 7 segundos sigue sin haber, permite la entrada manual.
//
// - msg: detecciones recibidas desde el topic de YOLO 3D.
// - tracker: estructura que almacena la posicion y tiempo de la ultima deteccion valida.
// - now: timestamp actual, usado para calcular si ha pasado suficiente tiempo.
// - node: puntero al nodo ROS 2 usado para logs y funciones auxiliares.
// - arm_group: interfaz del grupo de movimiento del brazo
// - gripper_group: interfaz del grupo de movimiento de la pinza.
// - planning_scene_interface: interfaz de la escena de planificacion.
void detection_callback(const yolo_msgs::msg::DetectionArray::SharedPtr msg,
                        DetectionTracker &tracker,
                        rclcpp::Time now,
                        const std::shared_ptr<rclcpp::Node> &node,
                        moveit::planning_interface::MoveGroupInterface &arm_group,
                        moveit::planning_interface::MoveGroupInterface &gripper_group,
                        moveit::planning_interface::PlanningSceneInterface &planning_scene_interface)
{   
    // Si ya se completo la tarea, ignorar nuevas detecciones
    if (task_done) return;

    // Verificar si hay alguna deteccion valida segun el mapa de objetos conocidos
    bool has_valid_detection = false;
    for (const auto &det : msg->detections) {
        if (kObjectShapes.find(det.class_name) != kObjectShapes.end()) {
            has_valid_detection = true;
            break;
        }
    }

    // En caso de que no reciban detecciones validas
    if (msg->detections.empty() || !has_valid_detection) {
        RCLCPP_INFO(node->get_logger(), "No detections found");

        // Si la cabeza no se ha inclinado aun, comenzar temporizador de 5 segundos
        if (!head_tilted) {
            if (!detection_timer_started) {
                last_detection_time = now;
                detection_timer_started = true;
            } 
            else if ((now - last_detection_time).seconds() >= 5.0) {
                RCLCPP_INFO(node->get_logger(), "No detections for 5 seconds. Tilting head down");
                tilt_head(node, 'd');
                head_tilted = true;
                detection_timer_started = false;
                second_timer_started = true;
                second_detection_time = now;
            }
        }

        // Si ya esta inclinada y tampoco hay detecciones tras 7 segundos, entrada manual del objeto o finalizar el nodo
        else if (second_timer_started && (now - second_detection_time).seconds() >= 7.0) {
            if (!rclcpp::ok()) return;
            second_timer_started = false;
            std::string object_class;
            std::cout << "No detections after tilting head down. Manually introduce object to manipulate (supported: can, bottle, book_front, book_side, cube and bag) or exit (type 'quit'): ";
            std::cin >> object_class;

            if (object_class == "quit") {
                RCLCPP_INFO(node->get_logger(), "Exited.");
                rclcpp::shutdown();
                return;
            }

            // Verificar si la clase manual es valida
            if (kObjectShapes.find(object_class) == kObjectShapes.end()) {
                RCLCPP_WARN(node->get_logger(), "Class not supported. Exited");
                rclcpp::shutdown();
                return;
            }
            
            // Solicitar posicion manualmente
            if (!rclcpp::ok()) return;
            double manual_x, manual_y, manual_z;
            std::cout << "Introduce object position (x y z): ";
            std::cin >> manual_x >> manual_y >> manual_z;
            while (rclcpp::ok() && (manual_x < 0.70 || manual_x > 1.00 || manual_y < -0.50 || manual_y > 0.50 || manual_z < 0.83)) 
            {        
                std::cout << "Position not valid. Try a new one (x y z): ";
                std::cin >> manual_x >> manual_y >> manual_z;
            }
            // Lanzar manipulacion con el objeto introducido manualmente
            manipulate_object(node, arm_group, gripper_group, planning_scene_interface, object_class, manual_x, manual_y, manual_z);
        }

        return;
    }


    // En caso de que se reciban detecciones validas
    // Se reinician los temporizadores de ausencia de detecciones
    detection_timer_started = false;
    second_timer_started = false;

    // Procesar las detecciones recibidas
    for (const auto &det : msg->detections) {
        auto it = kObjectShapes.find(det.class_name);
        if (it == kObjectShapes.end()) {
            RCLCPP_INFO(node->get_logger(), "No detections");
            continue;
        }

        // Obtener la posicion 3D del centro del objeto
        const auto &p = det.bbox3d.center.position;

        // Si ya habia una deteccion anterior, comprobar si es estable
        if (tracker.initialized) {
            double dx = std::abs(p.x - tracker.last_position.x);
            double dy = std::abs(p.y - tracker.last_position.y);
            double dz = std::abs(p.z - tracker.last_position.z);

            // Si el objeto no se ha movido, comprobar si han pasado 5 segundos
            if (dx < 0.05 && dy < 0.05 && dz < 0.05) {
                auto elapsed = now - tracker.last_time;
                if (elapsed.seconds() >= 5.0) {
                    RCLCPP_INFO(node->get_logger(), "Stable detection for 5 seconds. Proceeding to grasp");
                    
                    // Ajustar la clase del libro segun sus dimensiones, puede estar lateralmente o frontalmente
                    std::string object_class = det.class_name;
                    if (object_class == "book")
                    {
                        object_class = (det.bbox3d.size.y > det.bbox3d.size.x) ? "book_side" : "book_front";
                    }
                    
                    // Lanzar manipulacion
                    manipulate_object(node, arm_group, gripper_group, planning_scene_interface, object_class, p.x, p.y, p.z);
                    return;
                }
            } 
            // Si se ha movido, actualizar posicion y tiempo    
            else {
                tracker.last_position = p;
                tracker.last_time = now;
            }
        } 
        // En caso de que no se hubiese detectado antes y sea la primera deteccion valida, se almacena
        else {
            tracker.last_position = p;
            tracker.last_time = now;
            tracker.initialized = true;
        }

        break; // Solo se procesa una deteccion valida por callback
    }
}


// Funcion que crea e inicializa un grupo de movimiento de MoveIt para un conjunto de articulaciones especifico.
//
// - node: puntero al nodo ROS 2.
// - group_name: nombre del grupo de movimiento
//
// - devuelve: objeto MoveGroupInterface configurado para ese grupo.
moveit::planning_interface::MoveGroupInterface create_move_group(const std::shared_ptr<rclcpp::Node> &node,
                                                                 const std::string &group_name)
{   
    moveit::planning_interface::MoveGroupInterface move_group(node, group_name);
    move_group.setPoseReferenceFrame("base_footprint");
    move_group.setEndEffectorLink("gripper_grasping_frame");
    move_group.setMaxVelocityScalingFactor(0.65);
    move_group.setMaxAccelerationScalingFactor(0.50);
    move_group.setPlanningTime(30.0);
    move_group.setNumPlanningAttempts(30);
    
    return move_group;

}


// Funcion principal del nodo ROS 2. Inicializa los componentes necesarios, configura el planificador y ejecuta el bucle principal.
// - Crea los grupos de movimiento del brazo y la pinza.
// - Configura la escena de colision con la mesa.
// - Se suscribe al topic de detecciones 3D de YOLO.
// - Ejecuta la logica de manipulacion mientras la tarea no este finalizada.
int main(int argc, char *argv[])
{
    // Inicializar ROS 2 y crear nodo
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("mani_yolo_node");
    auto logger = node->get_logger();

    // Crear ejecutor multihilo
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    // Crear grupos de movimiento del brazo y pinza
    auto arm_group = create_move_group(node, "arm_torso");
    auto gripper_group = create_move_group(node, "gripper");
    gripper_group.setMaxVelocityScalingFactor(1.0);
    gripper_group.setMaxAccelerationScalingFactor(1.0);

    // Crear escena de colision con la mesa
    auto planning_scene_interface = create_scene(arm_group);
    RCLCPP_INFO(logger, "Planning scene created");

    // Inicializar estructura para seguimiento de detecciones estables
    DetectionTracker tracker;

    // Suscribirse al topic de detecciones 3D de YOLO
    auto sub = node->create_subscription<yolo_msgs::msg::DetectionArray>(
        "/yolo/detections_3d", 10,
        [&tracker, node, &arm_group, &gripper_group, &planning_scene_interface](const yolo_msgs::msg::DetectionArray::SharedPtr msg) {
            detection_callback(msg, tracker, rclcpp::Clock().now(), node, arm_group, gripper_group, planning_scene_interface);
    });

    // Ejecutar el bucle principal mientras la tarea no haya terminado
    rclcpp::Rate rate(10);
    while (rclcpp::ok() && !task_done) {
        executor.spin_some();
        rate.sleep();
    }

    // Finalizar el nodo de forma limpia
    RCLCPP_INFO(logger, "Shutting down node");
    executor.cancel();
    node.reset();
    rclcpp::shutdown();
    
    return 0;
}
