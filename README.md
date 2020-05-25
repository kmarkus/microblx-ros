# Microblx - ROS connector blocks

This repository contains the `ubxros` function block that can be used
to connect a microblx composition to ROS topics.

## Installing

**Dependencies**

To build this package, the following dependencies need to be
installed:

- ROS1 (tested with Debian buster)

- KDL (optional, if not found no support for `geometry_msgs` will be
  available).

- microblx-kdl-types
  ([git](https://github.com/kmarkus/microblx-kdl-types)). This adds
  support for basic KDL types in microblx.

**Building**

```sh
$ git clone

```

## Usage

`ubxros` is a /dynamic interface block/. This means that interface
depends on the block configuration. More specifically,

1.


## Design Rationale

It is tempting to realise the `ubxros` connector block as an microblx
iblock that reads/writes to the respective topic. However, this would
imply that the

## Acknowledgement

COCORF-ITP
