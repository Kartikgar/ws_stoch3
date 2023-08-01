# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "pronto_core;pronto_quadruped;geometry_msgs;sensor_msgs;pronto_msgs;visualization_msgs;eigen_conversions".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lpronto_quadruped_ros".split(';') if "-lpronto_quadruped_ros" != "" else []
PROJECT_NAME = "pronto_quadruped_ros"
PROJECT_SPACE_DIR = "/home/kartik/ws_stoch3/install"
PROJECT_VERSION = "0.3.0"
