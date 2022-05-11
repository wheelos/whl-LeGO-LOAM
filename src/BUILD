load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
  name = "image_projection",
  srcs = [
    "image_projection.cc",
  ],
  hdrs = [
    "//modules/tools/ilego_loam/include/utility.h",
    "//modules/tools/ilego_loam/include/image_projection.h",
  ],
  deps = [
    "//lego_loam_gflags",
    "//cloud_info",
  ],
)

cpplint()
