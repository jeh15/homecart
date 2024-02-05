# BUILD
cc_binary(
    name = "right_gripper_close",
    srcs = ["right_gripper_close.cc",],
    linkstatic = True,
    linkopts = ["-lpthread"],
    deps = [
        "@nlohmann_json//:json",
        "@drake//lcmtypes:schunk",
        "@gflags//:gflags",
        "@lcm//:lcm",
    ]
)
