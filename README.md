# Graph.py (ROS)

### Install

Make sure the following are installed **for python 3**

1. psutil
2. pandas
3. seaborn

## Example

With roscore already running, run

```sh
rosrun ltu_actor_graph graph.py --run example
```

or simply

```sh
roslaunch ltu_actor_graph example.launch
```

which will produce this graph:

![](doc/example.png)

graph.py has nothing to do with ROS other than it knows how to open a subprocess of rosrun (graph.py is **not** a ROS node). Internally graph.py will call `rosrun ltu_actor_graph <run>` with the argument passed to it.

The node that gets run is assumed to output to stdout in CSV format, with the first line being the graph title, and the first column of CSV data is the X axis.

## Example Code & Output format

```c++
#include <cmath>
#include <iostream>

int
main(int argc, char **argv)
{
    std::cout << "Graph Title" << std::endl;
    std::cout << "time,sin(x),cos(x),tan(x)" << std::endl;

    for (double x = 0; x < 1; x += 0.01)
        std::cout << x << ',' << sin(x) << ',' << cos(x) << ',' << tan(x) << std::endl;

    return EXIT_SUCCESS;
}
```

```CSV
Graph Title
time,sin(x),cos(x),tan(x)
0,0,1,0
0.01,0.00999983,0.99995,0.0100003
0.02,0.0199987,0.9998,0.0200027
0.03,0.0299955,0.99955,0.030009
0.04,0.0399893,0.9992,0.0400213
0.05,0.0499792,0.99875,0.0500417
0.06,0.059964,0.998201,0.0600721
0.07,0.0699428,0.997551,0.0701146
             etc.
```

## Launch file examples

Period is in ms.

#### twist
```xml
<launch>
  <node pkg="ltu_actor_graph" type="graph.py" name="graph" args="--run twist">
    <param name="input" value="/twist/topic" />
    <param name="period" value="10" />
  </node>
</launch>
```

#### std\_msgs
```xml
<launch>
  <node pkg="ltu_actor_graph" type="graph.py" name="graph" args="--run std_msgs">
    <param name="period" value="10" />
    <param name="type" value="Float32" />
    <param name="input" value="/twist/topic" />
  </node>
</launch>
```

