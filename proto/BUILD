load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("//tools:python_rules.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "cloud_info_cc_proto",
    deps = [
        ":cloud_info_proto",
    ],
)

proto_library(
    name = "cloud_info_proto",
    srcs = ["cloud_info.proto"],
)
