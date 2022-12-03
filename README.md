# Детальное объяснение настройки навигационного стека ROS1 move_base

Изображение MoveBase

## Пошаговая инструкция

<details>
<summary>
<b>1.Настройка карты преград (<a href="Sprint_1/D.cpp">juggle.cpp</a></b>)
</summary>

- `footprint:` - границы робота, задаются координатами относительно base_link в метрах
- `publish_frequency:` - частота публикации в Гц
- `transform_tolerance:` - Задает допустимую задержку преобразования данных (tf) в секундах. Этот параметр служит защитой от потери ссылки в дереве tf, в то же время позволяя существовать в системе с задержкой, которая устраивает пользователя. Если преобразование tf между кадрами координат, указанными параметрами global_frame и robot_base_frame, на несколько секунд старше transform_tolerance, чем ros::Time::now(), то стек навигации остановит робота.
- `map_type:` - costmap

```
footprint: [[0.40,0.225], [0.40,-0.225], [-0.50,-0.225], [-0.50,0.225]]
publish_frequency: 10.0
transform_tolerance: 1.0
map_type: costmap
```

<details>
<summary> 
<b>inflation_layer (<a href="Sprint_1/D.cpp">juggle.cpp</a></b>)
</summary>

`Инфляция` - это процесс распространения значений затрат из занятых ячеек, которые уменьшаются с расстоянием. Для этой цели мы определяем 5 конкретных символов для значений карты затрат, поскольку они относятся к роботу.

- `Lethal` стоимость означает, что в ячейке существует фактическое препятствие (рабочее пространство). Таким образом, если бы центр робота находился в этой ячейке, робот, очевидно, столкнулся бы.
- `Inscribed` - стоимость означает, что ячейка находится на расстоянии меньше вписанного радиуса робота от фактического препятствия. Таким образом, робот, безусловно, сталкивается с каким-то препятствием, если центр робота находится в ячейке, которая равна или превышает указанную стоимость.
- `Possibly circumscribed` - Стоимость аналогична стоимости `Inscribed`, но с использованием ограниченного радиуса робота в качестве расстояния отсечения. Таким образом, если центр робота находится в ячейке на уровне или выше этого значения, то от ориентации робота зависит, столкнется ли он с препятствием или нет. Мы используем термин "возможно", потому что, возможно, на самом деле это не ячейка препятствий, а какие-то предпочтения пользователя, которые определяют эту конкретную стоимость на карте. Например, если пользователь хочет указать, что робот должен попытаться обойти определенную область здания, он может включить свои собственные затраты в карту затрат для этого региона независимо от каких-либо препятствий. `Важное примечание:` Уровень инфляции навигации не обеспечивает соблюдение этой границы, возможно, вписанной в стоимость 128, однако относительно легко получить стоимость на этом расстоянии для использования в расчетах обнаружения столкновений.
- `Freespace` - Предполагается, что стоимость равна нулю, и это означает, что нет ничего, что могло бы помешать роботу отправиться туда.
- `Unknown` - стоимость означает, что нет никакой информации о данной ячейке. Пользователь карты затрат может интерпретировать это так, как считает нужным.
- Всем остальным затратам присваивается значение между `Freespace` и `Possibly circumscribed` в зависимости от их расстояния до `Lethal` ячейки и функции распада, предоставленной пользователем.
  Обоснование этих определений заключается в том, что мы оставляем за реализацией планировщика право заботиться или не заботиться о точном следе, но при этом предоставляем им достаточно информации, чтобы они могли понести расходы на отслеживание следа только в ситуациях, когда ориентация действительно имеет значение.

- `inflation_radius (double, default: 0.55)` - Радиус в метрах, до которого карта увеличивает значения стоимости препятствий

- `cost*scaling_factor (double, default: 10.0)` - Коэффициент масштабирования, применяемый к значениям затрат во время инфляции. Функция затрат вычисляется следующим образом для всех ячеек карты затрат дальше, чем расстояние по вписанному радиусу, и ближе, чем расстояние по радиусу инфляции, от фактического препятствия exp(-1.0 \* cost\*scaling_factor \* (distance_from_obstacle - inscribed_radius)) \* (costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1)
  где costmap_2d::INSCRIBED_INFLATED_OBSTACLE в настоящее время составляет 254. ПРИМЕЧАНИЕ: поскольку коэффициент cost_scaling_factor в формуле умножается на отрицательное значение, увеличение коэффициента приведет к уменьшению результирующих значений затрат

```
global_inflation_layer:
  enabled: true
  cost_scaling_factor: 1.75
  inflation_radius: 0.5

local_inflation_layer:
  enabled: true
  cost_scaling_factor: 5.0
  inflation_radius: 0.25
```

</details>

<details>
    <summary>
    <b>static_layer (<a href="Sprint_1/D.cpp">juggle.cpp</a></b>)
    </summary>
    Статическая карта включает в себя в основном неизменяемые данные из внешнего источника

- `unknown_cost_value (int, default: -1)` - Значение, для которого стоимость должна считаться неизвестной при считывании на карте с mapserver. Если карта затрат не отслеживает неизвестное пространство, затраты этого значения будут считаться занятыми. Нулевое значение также приводит к тому, что этот параметр не используется.
- `lethal_cost_threshold (int, default: 100)` - Пороговое значение, при котором стоимость считается смертельной при чтении карты с mapserver.
- `map_topic (string, default: "map")` - Топик на который подписывается карта затрат для статической карты. Этот параметр полезен, когда у вас есть несколько экземпляров costmap в пределах одной ноды, для которых вы хотите использовать разные статические карты.
- `first_map_only (bool, default: false)` - Подписывайтесь только на первое сообщение в теме карты, игнорируя все последующие сообщения
- `subscribe_to_updates (bool, default: false)` - В дополнение к map_topic, также подпишитесь на map_topic + "\_updates"
- `track_unknown_space (bool, default: true)` - Если значение true, неизвестные значения в сообщениях карты преобразуются непосредственно в слой. В противном случае неизвестные значения в сообщении карты переводятся как свободное пространство в слое.
- `use_maximum (bool, default: false)` - Имеет значение только в том случае, если статический слой не является нижним слоем. Если значение равно true, в основную карту затрат будет записано только максимальное значение.
- `trinary_costmap (bool, default: true)` -Если true, преобразует все значения сообщения карты в три значения NO_INFORMATION/FREE_SPACE/LETHAL_OBSTACLE. Если значение равно false, возможен полный спектр промежуточных значений.

```
static_layer:
  unknown_cost_value: -1
  lethal_cost_threshold: 254
  map_topic: "map"
  first_map_only: true
  subscribe_to_updates: false
  track_unknown_space: true
  use_maximum: true
  trinary_costmap: true
```

</details>

<details>
<summary>
<b>obstacle_layer (<a href="Sprint_1/D.cpp">juggle.cpp</a></b>)
</summary>

Слои препятствий и вокселей содержат информацию от датчиков в виде облаков точек или лазерных сканирований. Слой препятствий отслеживается в двух измерениях, в то время как слой вокселей отслеживается в трех.

### Marking and Clearing

Карта затрат автоматически подписывается на разделы датчиков и соответствующим образом обновляется. Каждый датчик используется либо для пометки (вставки информации о препятствии в карту затрат), либо для очистки (удаления информации о препятствии из карты затрат), либо и для того, и для другого. Операция очистки выполняет трассировку лучей по сетке от источника датчика наружу для каждого сообщенного наблюдения. С помощью воксельного слоя информация о препятствиях из каждого столбца проецируется вниз в двух измерениях при вводе в карту затрат.

### Subscribed Topics

- `point_cloud_topic (sensor_msgs/PointCloud)`
- `point_cloud2_topic (sensor_msgs/PointCloud2)`
- `point_cloud2_topic (sensor_msgs/PointCloud2)`
- `map" (nav_msgs/OccupancyGrid)`

### Sensor management parameters

- `observation_sources (string, default: "")`-
- `topic (string, default: source_name)`-
- `sensor_frame (string, default: "")`-
- `sensor_frame (string, default: "")`-
- `expected_update_rate (double, default: 0.0)`-
- `data_type (string, default: "PointCloud")`-
- `clearing (bool, default: false)`-
- `clearing (bool, default: false)`-
- `max_obstacle_height (double, default: 2.0)`-
- `min_obstacle_height (double, default: 0.0)`-
- `obstacle_range (double, default: 2.5)`-
- `raytrace_range (double, default: 3.0)`-
- `inf_is_valid (bool, default: false)`-

### Global Filtering Parameters

- `max_obstacle_height (double, default: 2.0)`-
- `obstacle_range (double, default: 2.5)`-
- `raytrace_range (double, default: 3.0)`-

### ObstacleCostmapPlugin

- `track_unknown_space (bool, default: false)`-
- `footprint_clearing_enabled (bool, default: true)`-
- `combination_method (enum, default: 1)`-

### VoxelCostmapPlugin

- `origin_z (double, default: 0.0)`-
- `z_resolution (double, default: 0.2)`-
- `z_voxels (int, default: 10)`-
- `unknown_threshold (int, default: - z_voxels)`-
- `mark_threshold (int, default: 0)`-
- `publish_voxel_map (bool, default: false)`-
- `footprint_clearing_enabled (bool, default: true)`-

```
laser_obstacle_layer:
    observation_sources: laser_scan  lidar_clear
    track_unknown_space: true
    footprint_clearing_enabled: true
    combination_method: 1  # 0 - overwrite, 1 - max, 99 - nothing

    laser_scan:
        sensor_frame: velodyne
        data_type: PointCloud2
        topic: /velodyne_points
        marking: true
        clearing: false
        obstacle_range: 10.
        raytrace_range: 12.
        inf_is_valid: true
        min_obstacle_height: 0.4
        max_obstacle_height: 1.0
        expected_update_rate: 10.0
        observation_persistence: 0.0
        inf_is_valid: true
        voxel_filter: true
        clear_after_reading: true

    lidar_clear:
        enabled: true
        data_type: PointCloud2
        topic: /velodyne_points
        marking: false
        clearing: true
        max_z: 2.0
        min_z: 0.1
        vertical_fov_angle: 0.523
        vertical_fov_padding: 0.05
        horizontal_fov_angle: 6.29
        decay_acceleration: 50.0
        model_type: 1
```

```
depth_obstacle_layer:
    observation_sources: point_cloud_kinect  point_clear
    track_unknown_space: true
    footprint_clearing_enabled: true
    combination_method: 1  # 0 - overwrite, 1 - max, 99 - nothing

    point_cloud_kinect:
        sensor_frame: camera_link
        data_type: PointCloud2
        topic: /filtered_obstacles
        marking: true
        clearing: false
        min_obstacle_height: 0.1
        max_obstacle_height: 0.6
        obstacle_range: 4.
        raytrace_range: 5.
        expected_update_rate: 10.0
        observation_persistence: 0.0
        inf_is_valid: true
        voxel_filter: true
        clear_after_reading: true

    point_clear:
        enabled: false
        data_type: PointCloud2
        topic: /filtered_ground_plane
        marking: false
        clearing: true
        max_z: 2.0
        min_z: -1.01
        #vertical_fov_angle: 0.523
        #vertical_fov_padding: 0.05
        #horizontal_fov_angle: 6.29
        #decay_acceleration: 5.0
        model_type: 0
```

</details>

</details>

<details>
<summary>
<b>2 Настройка карты глобальных преград (<a href="Sprint_1/D.cpp">juggle.cpp</a></b>)
</summary>
TExt
</details>

<details>
<summary>
<b>3 Настройка глобального планировщика (<a href="Sprint_1/D.cpp">juggle.cpp</a></b>)
</summary>
TExt
</details>
<details>
<summary>
<b>4 Настройка карты локальных преград (<a href="Sprint_1/D.cpp">juggle.cpp</a></b>)
</summary>
TExt
</details>
<details>
<summary>
<b>5 LocalPLanner (<a href="Sprint_1/D.cpp">juggle.cpp</a></b>)
</summary>
TExt
</details>
