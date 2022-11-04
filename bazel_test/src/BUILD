# BUILD
package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "demo",
    srcs = ["demo.cc",],
    linkstatic = 1,
    deps = [
        "@ur_rtde//:ur_rtde",
        "@drake//lcmtypes:schunk",
        "@gflags//:gflags",
        "@lcm//:lcm",
        ],
    linkopts = [
        "-lpthread",
        ],
)