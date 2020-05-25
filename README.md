# Microblx - ROS connector blocks

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [Microblx - ROS connector blocks](#microblx---ros-connector-blocks)
    - [Installing](#installing)
    - [Usage](#usage)
    - [Design Rationale](#design-rationale)
    - [Limitations](#limitations)
    - [Acknowledgement](#acknowledgement)

<!-- markdown-toc end -->


This repository contains the `ubxros` function block that can be used
to connect a microblx composition to ROS topics.

## Installing

**Dependencies**

To build this package, the following dependencies need to be
installed:

- microblx (at least v0.8.1)

- ROS1 (tested with Debian buster)

- KDL (e.g. `liborocos-kdl-dev`, optional, if not found no support for
  `geometry_msgs` will be available).

- microblx-kdl-types
  ([git](https://github.com/kmarkus/microblx-kdl-types)). This adds
  support for basic KDL types in microblx.

**Building**

```sh
$ git clone git@github.com:kmarkus/microblx-ros.git
...
$ cd microblx-ros
$ mkdir build
$ cd build
$ cmake ../
$ make 
[ 20%] Generating hexarr/types/ubxros_conn.h.hexarr
Scanning dependencies of target ubxros
[ 40%] Building CXX object CMakeFiles/ubxros.dir/src/ros_std.cpp.o
[ 60%] Building CXX object CMakeFiles/ubxros.dir/src/ros_kdl.cpp.o
[ 80%] Building CXX object CMakeFiles/ubxros.dir/src/ubxros.cpp.o
[100%] Linking CXX shared library ubxros.so
[100%] Built target ubxros
$ sudo make install
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

`ubxros` is a /dynamic interface block/, thus depending on its
configuration it will create a ports to pub/sub to/from ROS topics.

The complete, minimal example can be found here:
[example.usc](./usc/example.usc).

**Configuration**

The key configuration is `connections`, which is an array of `struct
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

**Connecting**

With the above configuration, upon initialization the `ubxros` block
will create ports named according to the topic. Writing to the `P`
topic will publish the value on the respective topic. Conversely, if
values are received on the `S` topic, these will be simply output on
the respective port.

```Lua
connections = {
    { src="rand_double1.out", tgt="rosbridge1.rand_double" },
	{ src="rosbridge1.listen_int32", tgt="listen_int32_out" },
},
```

**Triggering**

For subscriber topics, ROS will invoke callbacks that will forward the
data on the respective ports. However for publishing data it is up to
the user to add an external trigger like a `ptrig` block to cause data
available on the publishing ports to be output.

**Running the example**

Make sure to run `ubx-log` in a separate window to catch any errors.

```sh
$ ubx-launch -c usc/example.usc
...

```

in another terminal

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

To test the subscriber, first publish some dummy data:

```sh
$ rostopic pub -r 10 /listen_int32 std_msgs/Int32 5150
...
```

And view the data exported to the mqueue:

```sh
$ ubx-mq list
be345fddda975b690acfd15b4e22d4e1  1    listen_int32_out

$ ubx-mq read listen_int32_out
5150
5150
5150
...
```

## Design Rationale

It is tempting to implement the ROS connector block as an microblx
iblock that forwards reads/writes to the respective topic. The major
downside of this would be that the writing/reading takes place in the
context of the reading or writing cblock, thereby impacting its
real-time behavior.

Hence, the `ubxros` block was developed as a standalone cblock. This
way, it can be decoupled from the hard real-time domain via lock-free
connections and be assigned it's own, lower prio activity.

## Limitations

- No support for the `MultiArray` message types yet. It's probably not
  difficult...

## License

[BSD-3-Clause](./LICENSE)

## Acknowledgement

- Parts of this early
  [microblx_ros_bridge](https://bitbucket.org/haianos/microblx_ros_bridge/src)
  were a useful source of inspiration.
  
- COCORF-ITP
