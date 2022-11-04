# BUILD
load("@rules_foreign_cc//foreign_cc:defs.bzl", "cmake")

package(default_visibility = ["//visibility:public"])

cmake(
    name = "ur_rtde",
    lib_source = "@ur_rtde//:all",
    out_headers_only = 1,
)

cc_binary(
    name = "demo",
    srcs = ["demo.cc",],
    linkstatic = 1,
    deps = [
        ":ur_rtde",
        "@drake//lcmtypes:schunk",
        "@gflags//:gflags",
        "@lcm//:lcm",
        ],
    linkopts = [
        "-lpthread",
        ],
)