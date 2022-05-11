load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("//tools:python_rules.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "cloud_info",
    deps = [
        ":cloud_info",
    ],
)

proto_library(
    name = "cloud_info",
    srcs = ["cloud_info.proto"],
    deps = [
        "//modules/tools/ilego_loam/proto:cloud_info",
    ],
)
