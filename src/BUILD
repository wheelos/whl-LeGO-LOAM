load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
  name = "image_projection",
  srcs = [
    "image_projection.cc",
  ],
  hdrs = [
    "utility.h",
    "image_projection.h",
  ],
  deps = [
    "//cyber",
    "//modules/drivers/proto:pointcloud_cc_proto",
    "//modules/tools/ilego_loam/flags:lego_loam_gflags",
    "//modules/tools/ilego_loam/proto:cloud_info_cc_proto",
    "@local_config_pcl//:pcl",
    "@eigen",
    "@opencv//:core",
  ],
)

cpplint()
