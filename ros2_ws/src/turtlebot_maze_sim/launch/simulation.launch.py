
"""Standalone Gazebo simulation for the TurtleBot3 maze (ROS 2 Jazzy).

All geometry is generated from the original SDFs at launch time.

The scale argument scales the world XY plane about (0, 0).
Robot size and physical Z dimensions remain unchanged.

Optional gates and obstacle can be enabled independently.

The ground truth map is generated from the static scaled maze
before spawning the robot, gates and optional obstacle.
"""

import atexit
import math
import os
import re
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
    OpaqueFunction,
    LogInfo,
)

from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ============================================================
# Gate reference positions (scale = 1.0)
# ============================================================

GATE_DEFAULTS = (
    ('gate_1', '0.25', '1.5', '-1.57079632679', '0.63'),
    ('gate_2', '-1.2', '0.47', '0.0', '0.35'),
    ('gate_3', '1.2', '-0.16', '3.14159265359', '0.35'),
    ('gate_4', '0.25', '-1.5', '1.57079632679', '0.35'),
)


# ============================================================
# Helpers
# ============================================================

def number(context, name, positive=False):
    raw = LaunchConfiguration(name).perform(context)

    try:
        value = float(raw)

        if not math.isfinite(value):
            raise ValueError()

        if positive and value <= 0.0:
            raise ValueError()

        return value

    except ValueError as exc:
        raise ValueError(
            f'Invalid launch argument {name}: {raw!r}'
        ) from exc


def scaled_number(context, name, scale, positive=False):
    value = number(context, name, positive=positive) * scale

    if not math.isfinite(value):
        raise ValueError(
            f'Scaled value of {name} is not finite'
        )

    return value


def enabled(context, name):
    return (
        LaunchConfiguration(name)
        .perform(context)
        .lower() in ('true', '1')
    )


def scale_vector(element, factors):
    if element is None or not element.text:
        raise ValueError('Required SDF vector is missing')

    values = [float(v) for v in element.text.split()]

    if len(values) != len(factors):
        raise ValueError(
            f'Unexpected vector: {element.text!r}'
        )

    result = [
        v * f for v, f in zip(values, factors)
    ]

    if not all(math.isfinite(v) for v in result):
        raise ValueError('Non-finite scaled geometry')

    element.text = ' '.join(str(v) for v in result)


def write_xml(tree, path):
    tree.write(
        path,
        encoding='utf-8',
        xml_declaration=True
    )


# ============================================================
# Maze scaling
# ============================================================

def scaled_maze_model(source, scale):
    tree = ET.parse(source)
    model = tree.getroot().find('model')

    if model is None or model.get('name') != 'maze':
        raise ValueError(
            f'Expected maze model in {source}'
        )

    # STL is Y-up and rotated +90 degrees about X.
    # STL X and Z correspond to world X and Y.

    for tag in ('visual', 'collision'):
        parts = model.findall(f'./link/{tag}')

        if not parts:
            raise ValueError(
                f'Missing maze {tag} geometry'
            )

        for part in parts:
            scale_vector(
                part.find('pose'),
                (scale, scale, 1, 1, 1, 1)
            )

            scale_vector(
                part.find('./geometry/mesh/scale'),
                (scale, 1, scale)
            )

    return model


# ============================================================
# Gate scaling
# ============================================================

def generate_gate(source, destination, scale):
    tree = ET.parse(source)
    model = tree.getroot().find('model')

    if model is None:
        raise ValueError(
            f'Missing gate model in {source}'
        )

    for tag in ('visual', 'collision'):
        parts = model.findall(f'./link/{tag}')

        if not parts:
            raise ValueError(
                f'Missing gate {tag} geometry'
            )

        for part in parts:
            scale_vector(
                part.find('pose'),
                (scale, scale, 1, 1, 1, 1)
            )

            scale_vector(
                part.find('./geometry/box/size'),
                (scale, scale, 1)
            )

    write_xml(tree, destination)


# ============================================================
# Entrance and exit closure walls
# ============================================================

def closure_wall(context, name, scale):
    x = scaled_number(context, f'{name}_x', scale)
    y = scaled_number(context, f'{name}_y', scale)

    yaw = number(context, f'{name}_yaw')

    length = scaled_number(
        context,
        f'{name}_length',
        scale,
        positive=True
    )

    size = f'{length} {0.03 * scale} 0.25'

    return ET.fromstring(f"""
    <model name='{name}'>
      <static>true</static>

      <pose>{x} {y} 0.125 0 0 {yaw}</pose>

      <link name='wall'>
        <collision name='collision'>
          <geometry>
            <box>
              <size>{size}</size>
            </box>
          </geometry>
        </collision>

        <visual name='visual'>
          <geometry>
            <box>
              <size>{size}</size>
            </box>
          </geometry>

          <material>
            <ambient>0.30 0.48 0.65 1</ambient>
            <diffuse>0.30 0.48 0.65 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    """)


# ============================================================
# NEW: Optional static obstacle
# ============================================================

def generate_obstacle(destination, scale, length, width, height):
    """Generate an independent static obstacle SDF.

    X and Y dimensions scale with the maze.
    Height remains unchanged.

    The model origin is at the center of the box.
    Its Z position is assigned at spawn time.
    """

    size_x = length * scale
    size_y = width * scale

    model = ET.fromstring(f"""
    <sdf version="1.9">
      <model name="maze_obstacle">

        <static>true</static>

        <link name="obstacle_link">

          <collision name="collision">
            <geometry>
              <box>
                <size>{size_x} {size_y} {height}</size>
              </box>
            </geometry>
          </collision>

          <visual name="visual">
            <geometry>
              <box>
                <size>{size_x} {size_y} {height}</size>
              </box>
            </geometry>

            <material>
              <ambient>0.85 0.35 0.10 1</ambient>
              <diffuse>0.85 0.35 0.10 1</diffuse>
            </material>
          </visual>

        </link>
      </model>
    </sdf>
    """)

    write_xml(
        ET.ElementTree(model),
        destination
    )


# ============================================================
# Gazebo GUI scaling
# ============================================================

def read_gui_xml(source):
    text = Path(source).read_text(
        encoding='utf-8-sig'
    )

    text = re.sub(
        r'^\s*<\?xml\s+[^?]*\?>',
        '',
        text,
        count=1
    )

    try:
        return ET.fromstring(text), False

    except ET.ParseError as first_error:
        try:
            return ET.fromstring(
                '<gui_fragment>' +
                text +
                '</gui_fragment>'
            ), True

        except ET.ParseError as exc:
            raise ValueError(
                f'Invalid GUI XML in {source}: {exc}'
            ) from first_error


def generate_gui(source, destination, scale):
    root, fragment = read_gui_xml(source)

    scene = next(
        (
            p for p in root.iter('plugin')
            if p.get('filename') == 'MinimalScene'
        ),
        None
    )

    if scene is None:
        raise ValueError(
            f'MinimalScene plugin not found in {source}'
        )

    pose = scene.find('camera_pose')

    if pose is None:
        pose = ET.SubElement(
            scene,
            'camera_pose'
        )
        pose.text = '0 0 2 0 1.5708 0'

    scale_vector(
        pose,
        (scale, scale, scale, 1, 1, 1)
    )

    if fragment:
        Path(destination).write_text(
            '\n'.join(
                ET.tostring(child, encoding='unicode')
                for child in root
            ) + '\n',
            encoding='utf-8'
        )
    else:
        write_xml(
            ET.ElementTree(root),
            destination
        )


# ============================================================
# Temporary scenario cleanup
# ============================================================

def cleanup_scenario(temp):
    try:
        shutil.rmtree(temp)

    except FileNotFoundError:
        pass

    except OSError as exc:
        print(
            f'Could not remove temporary scenario '
            f'{temp}: {exc}',
            flush=True
        )

    else:
        print(
            f'Removed temporary scenario: {temp}',
            flush=True
        )


# ============================================================
# Generate complete simulation scenario
# ============================================================

def generate_scenario(pkg, context, scale):

    # Validate gates only if gates are enabled.
    if enabled(context, 'gates'):
        for name, *_ in GATE_DEFAULTS:
            extension = number(
                context,
                f'{name}_extension',
                positive=True
            )

            if extension > 0.65:
                raise ValueError(
                    f'{name}_extension must be <= 0.65 m '
                    f'at scale 1.0'
                )

    # Gates and obstacle are independent.
    # No dependency checks are required.

    temp = Path(
        tempfile.mkdtemp(
            prefix='turtlebot_maze_sim_',
            dir='/tmp'
        )
    )

    atexit.register(cleanup_scenario, temp)

    world_file = temp / 'maze_world.sdf'
    gate_file = temp / 'gate_scaled.sdf'
    obstacle_file = temp / 'obstacle.sdf'
    gui_file = temp / 'gui.config'

    tree = ET.parse(
        pkg / 'worlds/maze_world.sdf'
    )

    world = tree.getroot().find('world')

    if world is None or world.get('name') != 'maze_world':
        raise ValueError(
            'Expected world named maze_world'
        )

    includes = [
        inc for inc in world.findall('include')
        if (
            inc.findtext('uri') or ''
        ).strip().rstrip('/') == 'model://maze'
    ]

    if len(includes) != 1:
        raise ValueError(
            'Expected exactly one model://maze include'
        )

    include = includes[0]

    include_pose = include.findtext('pose')

    if include_pose and any(
        float(v) != 0.0
        for v in include_pose.split()
    ):
        raise ValueError(
            'Maze include pose must be identity '
            'for XY scaling'
        )

    # Scale maze geometry.
    model = scaled_maze_model(
        pkg / 'models/maze/model.sdf',
        scale
    )

    world.remove(include)
    world.append(model)

    # Scale ground plane.
    for ground in world.findall(
        "./model[@name='ground_plane']"
    ):
        for size in ground.findall(
            './/geometry/plane/size'
        ):
            scale_vector(
                size,
                (scale, scale)
            )

    # Optional entrance / exit closure walls.
    if enabled(context, 'closed_maze'):
        for name in (
            'entrance_wall',
            'exit_wall'
        ):
            world.append(
                closure_wall(
                    context,
                    name,
                    scale
                )
            )

    # Write static maze world.
    # Dynamic gates and the optional obstacle
    # are NOT included in this world file.
    write_xml(tree, world_file)

    # Generate scaled gates.
    generate_gate(
        pkg / 'models/gates/model.sdf',
        gate_file,
        scale
    )

    # Generate obstacle only when requested.
    if enabled(context, 'obstacle'):
        generate_obstacle(
            obstacle_file,
            scale,
            number(
                context,
                'obstacle_length',
                positive=True
            ),
            number(
                context,
                'obstacle_width',
                positive=True
            ),
            number(
                context,
                'obstacle_height',
                positive=True
            )
        )

    # Scale GUI overview.
    generate_gui(
        pkg / 'config/gazebo/gui.config',
        gui_file,
        scale
    )

    return (
        temp,
        world_file,
        gate_file,
        obstacle_file,
        gui_file
    )


# ============================================================
# Ground truth map generation
# ============================================================

def generate_ground_truth_map(world_file, models_path):
    output_dir = Path(
        '/ros2_ws/src/turtlebot_maze_sim/maps/ground_truth'
    )

    output_dir.mkdir(
        parents=True,
        exist_ok=True
    )

    print(
        f'Generating ground truth map from: {world_file}',
        flush=True
    )

    subprocess.run([
        'ros2',
        'run',
        'sdf2map',
        'sdf2map',

        '--input',
        str(world_file),

        '--model-path',
        str(models_path),

        '--output',
        str(output_dir / 'maze_gt.pcd'),

        '--occupancy-grid',
        str(output_dir / 'maze_gt.yaml'),

        '--exclude-ground',

        '--geometry',
        'collision',

        '--density',
        '10000',

        '--grid-resolution',
        '0.03',

        '--slice-z-min',
        '0.10',

        '--slice-z-max',
        '0.20',

        '--grid-bounds',
        'slice',

        '--grid-margin',
        '0.10',

        '--grid-close',
        '1',

        '--verbose',
    ], check=True)

    print(
        f'Ground truth map generated in: {output_dir}',
        flush=True
    )


# ============================================================
# Launch setup
# ============================================================

def launch_setup(context, *args, **kwargs):

    pkg_sim = get_package_share_directory(
        'turtlebot_maze_sim'
    )

    pkg_ros_gz_sim = get_package_share_directory(
        'ros_gz_sim'
    )

    pkg_tb3_gazebo = get_package_share_directory(
        'turtlebot3_gazebo'
    )

    models_path = os.path.join(
        pkg_sim,
        'models'
    )

    robot_sdf = os.path.join(
        models_path,
        'turtlebot3_burger',
        'model.sdf'
    )

    bridge_file = os.path.join(
        pkg_sim,
        'config',
        'bridges',
        'gazebo_bridge.yaml'
    )

    gt_bridge_file = os.path.join(
        pkg_sim,
        'config',
        'bridges',
        'ground_truth_bridge.yaml'
    )

    scale = number(
        context,
        'scale',
        positive=True
    )

    (
        temp_dir,
        world_file,
        gate_sdf,
        obstacle_sdf,
        gui_config
    ) = generate_scenario(
        Path(pkg_sim),
        context,
        scale
    )

    # The ground truth map is generated BEFORE
    # spawning the robot, gates or obstacle.
    if enabled(context, 'generate_map'):
        generate_ground_truth_map(
            world_file,
            models_path
        )

    prior_resources = os.environ.get(
        'GZ_SIM_RESOURCE_PATH',
        ''
    )

    resource_path = os.pathsep.join(
        p for p in (
            models_path,
            prior_resources
        )
        if p
    )

    ground_truth = LaunchConfiguration(
        'ground_truth'
    )

    gates = LaunchConfiguration(
        'gates'
    )

    obstacle = LaunchConfiguration(
        'obstacle'
    )

    # --------------------------------------------------------
    # Gazebo
    # --------------------------------------------------------

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_ros_gz_sim,
                'launch',
                'gz_sim.launch.py'
            )
        ),

        launch_arguments={
            'on_exit_shutdown': 'true',
            'gz_args': (
                f'-r -v 3 {world_file} '
                f'--gui-config {gui_config}'
            )
        }.items()
    )

    # --------------------------------------------------------
    # Robot State Publisher
    # --------------------------------------------------------

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_tb3_gazebo,
                'launch',
                'robot_state_publisher.launch.py'
            )
        ),

        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )

    # --------------------------------------------------------
    # Spawn TurtleBot3 Burger
    # --------------------------------------------------------

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',

        arguments=[
            '-name', 'burger',
            '-file', robot_sdf,

            '-x', str(scaled_number(
                context, 'x_pose', scale
            )),

            '-y', str(scaled_number(
                context, 'y_pose', scale
            )),

            '-z', LaunchConfiguration(
                'z_pose'
            ),

            '-Y', LaunchConfiguration(
                'yaw_pose'
            ),
        ],

        output='screen'
    )

    # --------------------------------------------------------
    # ROS / Gazebo Bridge
    # --------------------------------------------------------

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gazebo_bridge',

        parameters=[{
            'config_file': bridge_file
        }],

        output='screen'
    )

    # --------------------------------------------------------
    # Ground truth bridge
    # --------------------------------------------------------

    gt_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ground_truth_bridge',

        parameters=[{
            'config_file': gt_bridge_file
        }],

        condition=IfCondition(
            ground_truth
        ),

        output='screen'
    )

    # --------------------------------------------------------
    # Gate controller
    # --------------------------------------------------------

    gate_controller = Node(
        package='turtlebot_maze_sim',
        executable='random_gates.py',
        name='random_gates',

        parameters=[{
            'use_sim_time': True,

            'speed_mps': number(
                context, 'gate_speed'
            ),

            'world_name': 'maze_world',

            'update_rate_hz': number(
                context, 'gate_rate'
            ),

            'service_timeout_ms': 1200,

            'max_extension_limit': 0.65 * scale,

            'gate_1_x': scaled_number(
                context, 'gate_1_x', scale
            ),
            'gate_1_y': scaled_number(
                context, 'gate_1_y', scale
            ),
            'gate_1_z': number(
                context, 'gate_1_z'
            ),
            'gate_1_yaw': number(
                context, 'gate_1_yaw'
            ),
            'gate_1_extension': scaled_number(
                context, 'gate_1_extension', scale
            ),

            'gate_2_x': scaled_number(
                context, 'gate_2_x', scale
            ),
            'gate_2_y': scaled_number(
                context, 'gate_2_y', scale
            ),
            'gate_2_z': number(
                context, 'gate_2_z'
            ),
            'gate_2_yaw': number(
                context, 'gate_2_yaw'
            ),
            'gate_2_extension': scaled_number(
                context, 'gate_2_extension', scale
            ),

            'gate_3_x': scaled_number(
                context, 'gate_3_x', scale
            ),
            'gate_3_y': scaled_number(
                context, 'gate_3_y', scale
            ),
            'gate_3_z': number(
                context, 'gate_3_z'
            ),
            'gate_3_yaw': number(
                context, 'gate_3_yaw'
            ),
            'gate_3_extension': scaled_number(
                context, 'gate_3_extension', scale
            ),

            'gate_4_x': scaled_number(
                context, 'gate_4_x', scale
            ),
            'gate_4_y': scaled_number(
                context, 'gate_4_y', scale
            ),
            'gate_4_z': number(
                context, 'gate_4_z'
            ),
            'gate_4_yaw': number(
                context, 'gate_4_yaw'
            ),
            'gate_4_extension': scaled_number(
                context, 'gate_4_extension', scale
            ),

            'open_min_s': number(
                context, 'gate_open_min'
            ),

            'open_max_s': number(
                context, 'gate_open_max'
            ),

            'closed_min_s': number(
                context, 'gate_closed_min'
            ),

            'closed_max_s': number(
                context, 'gate_closed_max'
            ),

            'startup_delay_s': number(
                context, 'gate_startup_delay'
            ),

            'seed': int(
                LaunchConfiguration(
                    'gate_seed'
                ).perform(context)
            ),
        }],

        condition=IfCondition(
            gates
        ),

        output='screen'
    )

    # --------------------------------------------------------
    # IMU TF
    # --------------------------------------------------------

    imu_frame_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',

        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',

            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',

            '--frame-id', 'imu_link',
            '--child-frame-id',
            'burger/imu_link/tb3_imu',
        ],

        output='screen'
    )

    # --------------------------------------------------------
    # Ground truth odometry
    # --------------------------------------------------------

    ground_truth_odometry = Node(
        package='turtlebot_maze_sim',
        executable='ground_truth_odometry.py',
        name='ground_truth_odometry',

        parameters=[{
            'use_sim_time': True,
            'input_topic': '/ground_truth_pose',
            'output_topic': '/odom_ground_truth',
        }],

        condition=IfCondition(
            ground_truth
        ),

        output='screen'
    )

    # --------------------------------------------------------
    # Camera bridge
    # --------------------------------------------------------

    keyboard_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',

        arguments=[
            '/keyboard/keypress@std_msgs/msg/Int32[gz.msgs.Int32',

            '/world/maze_world/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],

        remappings=[
            (
                '/world/maze_world/pose/info',
                '/camera/world_poses'
            ),
        ],

        output='screen'
    )

    # --------------------------------------------------------
    # Camera controller
    # --------------------------------------------------------

    camera_controller = Node(
        package='turtlebot_maze_sim',
        executable='camera_controller.py',
        name='camera_controller',

        parameters=[{
            'robot_name': 'burger',
            'overview_config': str(gui_config),

            'follow_x': -0.45,
            'follow_y': 0.0,
            'follow_z': 0.35,
        }],

        output='screen'
    )

    # --------------------------------------------------------
    # Spawn gates
    # --------------------------------------------------------

    gate_nodes = []

    for name, *_ in GATE_DEFAULTS:

        gate_nodes.append(
            Node(
                package='ros_gz_sim',
                executable='create',
                name=f'spawn_{name}',

                arguments=[
                    '-name', name,
                    '-file', str(gate_sdf),

                    '-x', str(scaled_number(
                        context, f'{name}_x', scale
                    )),

                    '-y', str(scaled_number(
                        context, f'{name}_y', scale
                    )),

                    '-z', str(number(
                        context, f'{name}_z'
                    )),

                    '-Y', str(number(
                        context, f'{name}_yaw'
                    )),
                ],

                condition=IfCondition(
                    gates
                ),

                output='screen'
            )
        )

    # --------------------------------------------------------
    # NEW: Spawn optional obstacle
    # Independent from gates
    # --------------------------------------------------------

    obstacle_height = number(
        context,
        'obstacle_height',
        positive=True
    )

    obstacle_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_obstacle',

        arguments=[
            '-name', 'maze_obstacle',

            '-file', str(obstacle_sdf),

            '-x', str(scaled_number(
                context,
                'obstacle_x',
                scale
            )),

            '-y', str(scaled_number(
                context,
                'obstacle_y',
                scale
            )),

            '-z', str(obstacle_height / 2.0),

            '-Y', str(number(
                context,
                'obstacle_yaw'
            )),
        ],

        condition=IfCondition(
            obstacle
        ),

        output='screen'
    )

    # --------------------------------------------------------
    # Launch actions
    # --------------------------------------------------------

    return [
        LogInfo(
            msg=(
                f'Maze XY scale: {scale:g}; '
                f'generated scenario: {temp_dir}'
            )
        ),

        LogInfo(
            msg=(
                f'Gates enabled: '
                f'{enabled(context, "gates")}; '
                f'Obstacle enabled: '
                f'{enabled(context, "obstacle")}'
            )
        ),

        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            resource_path
        ),

        gz_sim,
        rsp,

        spawn_robot,

        *gate_nodes,

        obstacle_spawn,

        bridge,
        gt_bridge,

        TimerAction(
            period=5.0,
            actions=[gate_controller]
        ),

        imu_frame_transform,
        ground_truth_odometry,

        keyboard_bridge,
        camera_controller,
    ]


# ============================================================
# Launch arguments
# ============================================================

def generate_launch_description():

    args = [

        # ----------------------------------------------------
        # General simulation
        # ----------------------------------------------------

        DeclareLaunchArgument(
            'scale',
            default_value='1.0',
            description='Horizontal XY scale about world origin'
        ),

        DeclareLaunchArgument(
            'x_pose',
            default_value='-0.6',
            description='Robot reference X at scale 1.0'
        ),

        DeclareLaunchArgument(
            'y_pose',
            default_value='-0.9',
            description='Robot reference Y at scale 1.0'
        ),

        DeclareLaunchArgument(
            'z_pose',
            default_value='0.04'
        ),

        DeclareLaunchArgument(
            'yaw_pose',
            default_value='0.0'
        ),

        # ----------------------------------------------------
        # Maze closure
        # ----------------------------------------------------

        DeclareLaunchArgument(
            'closed_maze',
            default_value='false'
        ),

        DeclareLaunchArgument(
            'entrance_wall_x',
            default_value='-0.77'
        ),

        DeclareLaunchArgument(
            'entrance_wall_y',
            default_value='-0.9'
        ),

        DeclareLaunchArgument(
            'entrance_wall_yaw',
            default_value='1.5708'
        ),

        DeclareLaunchArgument(
            'entrance_wall_length',
            default_value='0.35'
        ),

        DeclareLaunchArgument(
            'exit_wall_x',
            default_value='0.77'
        ),

        DeclareLaunchArgument(
            'exit_wall_y',
            default_value='0.9'
        ),

        DeclareLaunchArgument(
            'exit_wall_yaw',
            default_value='1.5708'
        ),

        DeclareLaunchArgument(
            'exit_wall_length',
            default_value='0.35'
        ),

        # ----------------------------------------------------
        # Ground truth
        # ----------------------------------------------------

        DeclareLaunchArgument(
            'ground_truth',
            default_value='false'
        ),

        DeclareLaunchArgument(
            'generate_map',
            default_value='false',
            description='Generate the static ground truth map'
        ),

        # ----------------------------------------------------
        # Gates
        # ----------------------------------------------------

        DeclareLaunchArgument(
            'gates',
            default_value='false'
        ),

        DeclareLaunchArgument(
            'gate_speed',
            default_value='0.08',
            description='Peak gate speed in m/s, not scaled'
        ),

        DeclareLaunchArgument(
            'gate_rate',
            default_value='20.0'
        ),

        DeclareLaunchArgument(
            'gate_open_min',
            default_value='8.0'
        ),

        DeclareLaunchArgument(
            'gate_open_max',
            default_value='16.0'
        ),

        DeclareLaunchArgument(
            'gate_closed_min',
            default_value='5.0'
        ),

        DeclareLaunchArgument(
            'gate_closed_max',
            default_value='11.0'
        ),

        DeclareLaunchArgument(
            'gate_startup_delay',
            default_value='5.0'
        ),

        DeclareLaunchArgument(
            'gate_seed',
            default_value='-1'
        ),

        # ----------------------------------------------------
        # NEW: Optional independent obstacle
        # ----------------------------------------------------

        DeclareLaunchArgument(
            'obstacle',
            default_value='false',
            description='Spawn optional static obstacle'
        ),

        DeclareLaunchArgument(
            'obstacle_x',
            default_value='0.0',
            description='Obstacle center X at scale 1.0'
        ),

        DeclareLaunchArgument(
            'obstacle_y',
            default_value='-0.2',
            description='Obstacle center Y at scale 1.0'
        ),

        DeclareLaunchArgument(
            'obstacle_yaw',
            default_value='0.0',
            description='Obstacle yaw in radians'
        ),

        DeclareLaunchArgument(
            'obstacle_length',
            default_value='0.2',
            description='Local X dimension at scale 1.0'
        ),

        DeclareLaunchArgument(
            'obstacle_width',
            default_value='0.03',
            description='Local Y dimension at scale 1.0'
        ),

        DeclareLaunchArgument(
            'obstacle_height',
            default_value='0.25',
            description='Height in meters, not scaled'
        ),
    ]

    # --------------------------------------------------------
    # Gate-specific arguments
    # --------------------------------------------------------

    for name, x, y, yaw, extension in GATE_DEFAULTS:

        args.extend([

            DeclareLaunchArgument(
                f'{name}_x',
                default_value=x,
                description='Reference X at scale 1.0'
            ),

            DeclareLaunchArgument(
                f'{name}_y',
                default_value=y,
                description='Reference Y at scale 1.0'
            ),

            DeclareLaunchArgument(
                f'{name}_z',
                default_value='0.13'
            ),

            DeclareLaunchArgument(
                f'{name}_yaw',
                default_value=yaw
            ),

            DeclareLaunchArgument(
                f'{name}_extension',
                default_value=extension,
                description='Reference extension at scale 1.0'
            ),
        ])

    return LaunchDescription([
        *args,
        OpaqueFunction(
            function=launch_setup
        )
    ])
