load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
  name = "lego_loam_gflags",
  srcs = [
    "lego_loam_gflags.cc",
  ],
  hdrs = [
    "lego_loam_gflags.h",
  ],
  deps = [
    "@com_github_gflags_gflags//:gflags",
  ],
)

cpplint()
