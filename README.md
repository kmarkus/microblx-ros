# microblx - ROS connector block

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [Overview](#overview)
- [Installing](#installing)
- [Usage](#usage)
    - [Running the example](#running-the-example)
    - [Configuration](#configuration)
    - [Connecting](#connecting)
    - [Triggering](#triggering)
- [Design Rationale](#design-rationale)
- [Limitations](#limitations)
- [License](#license)
- [Acknowledgement](#acknowledgement)

<!-- markdown-toc end -->

## Overview

This repository contains the `ubxros` function block that can be used
to connect a microblx composition to ROS topics. The principal
use-case is to connect a minimal, hard real-time control or signal
processing core to larger non real-time ROS application.

This works as follows: the `ubxros` block is configured with the
topics you want to publish or subscribe to. During initialization, this
block creates microblx ports that are linked to the respective ROS
topics. This means messages received on sub topics are written to
output ports, and those read from input ports published to pub topics.

To preserve hard real-time properties of core blocks, it is important
to:

- connect to the `ubxros` ports using *lock-free* connections (this is
  the default for .usc compositions)
- trigger `ubxros` blocks with a *separate* (active) trigger like `ptrig`)

For more background information, please check out the [microblx
docs](microblx.readthedocs.io).

## Installing

**Dependencies**

To build this package, the following dependencies need to be
installed:

- microblx (at least v0.9.0)
- ROS1 (tested with Debian bullseye)
- KDL (e.g. `liborocos-kdl-dev` or from src)
- microblx-kdl-types
  ([git](https://github.com/kmarkus/microblx-kdl-types)). This adds
  support for basic KDL types in microblx.
  
The latter two are optional. If either of these is /not/ found, no
support for geometry msgs will be available.

**Building**

```sh
$ git clone git@github.com:kmarkus/microblx-ros.git
...
$ cd microblx-ros
$ mkdir build
$ cd build
$ cmake ../
...
building geometry_msgs support
...
$ make 
[ 20%] Generating hexarr/types/ubxros_conn.h.hexarr
Scanning dependencies of target ubxros
[ 40%] Building CXX object CMakeFiles/ubxros.dir/src/ros_std.cpp.o
[ 60%] Building CXX object CMakeFiles/ubxros.dir/src/ros_kdl.cpp.o
[ 80%] Building CXX object CMakeFiles/ubxros.dir/src/ubxros.cpp.o
[100%] Linking CXX shared library ubxros.so
[100%] Built target ubxros
$ sudo make install
...
```

check it's there:

```sh
$ ubx-modinfo show ubxros -p kdl_types
module ubxros
 license: BSD-3-Clause
 
  types:
    struct ubxros_conn, sz=296, nil
	
  blocks:
   ubxros [state: preinit, steps: 0] (type: cblock, prototype: false, attrs: )
     configs:
       connections [struct ubxros_conn] nil // microblx-ROS connections
```

## Usage

`ubxros` is a /dynamic interface block/ that according to its
configuration will create ports to publish/subscribe to/from ROS
topics

A complete, minimal example can be found here:
[example.usc](./usc/example.usc).

### Running the example

Before starting, run a `roscore`. Also make sure to run `ubx-log` in a
separate terminal to be able to see if anything goes wrong.

Now launch the composition:

```sh
$ ubx-launch -c usc/example.usc
...

```

**Testing the publisher**

```sh
$ rostopic list
/listen_int32
/rand_double
/rosout
/rosout_agg

$ rostopic echo /rand_double 
data: 0.8677190964591368
...
```

**Testing the subscriber**

To test the subscriber, first publish some dummy data:

```sh
$ rostopic pub -r 10 /listen_int32 std_msgs/Int32 5150
...
```

And view the data exported to the mqueue:

```sh
$ ubx-mq list
be345fddda975b690acfd15b4e22d4e1  1    listen_int32_out
...

$ ubx-mq read listen_int32_out
5150
5150
5150
...
```

### Configuration

The main configuration is `connections`, which is an array of `struct
ubxros_conn` types:

```lua
{
	 name="rosbridge1",
	 config = {
	    connections = {
	       { topic = "rand_double", dir='P', ubx_type="double" },
	       { topic = "listen_int32", dir='S', ubx_type="int32_t" },
	    }
	 }
},

```

Legal values are

```Lua
{ 
	topic = TOPIC_NAME,
	dir = P|S, 
	ubx_type = UBX_TYPE,
	queue_size = ROS_QUEUE_SIZE,
	latch = 0|1
},
```

See the roscpp documentation for the meaning of these attributes.

### Connecting

With the above configuration, upon initialization the `ubxros` block
will create ports named according to the topic. Writing to the `P`
ports will publish the value on the respective topic. Conversely, if
values are received on the `S` topic, these will be simply output on
the respective microblx port.

```Lua
connections = {
    { src="rand_double1.out", tgt="rosbridge1.rand_double" },
	{ src="rosbridge1.listen_int32", tgt="listen_int32_out" },
},
```

### Triggering

For subscriber topics, the `ubxros` block has setup callbacks that
directly output the data on the respective ports. Publishing to topics
takes place in the `ubxros` `step` hook. It is the responsibility of
the user to add a trigger block (e.g. `ptrig`) to periodically publish
output data.

## Design Rationale

It is tempting to implement the ROS connector block as an microblx
iblock that forwards reads/writes to the respective topic. The major
downside of this would be that the writing/reading takes place in the
context of the reading or writing cblock, thereby affecting its
real-time behavior.

To avoid this, the `ubxros` block was developed as a standalone
cblock. This way, it can be decoupled from the hard real-time domain
via lock-free connections and be triggered by a dedicated lower
priority trigger.

## Limitations

- No support for the `MultiArray` message types yet. It's probably
  straightforward to add these.

- No support for services or actions.

## License

[BSD-3-Clause](./LICENSE)

## Acknowledgement

- Parts of this early
  [microblx_ros_bridge](https://bitbucket.org/haianos/microblx_ros_bridge/src)
  were a useful source of inspiration.
  
- Development of microblx-ros was supported by the European H2020
  project RobMoSys via the COCORF (Component Composition for Real-time
  Function blocks) Integrated Technical Project.
