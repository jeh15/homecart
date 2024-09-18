# BUILD
cc_binary(
    name = "left_gripper_open",
    srcs = ["left_gripper_open.cc",],
    linkstatic = True,
    linkopts = ["-lpthread"],
    deps = [
        "@nlohmann_json//:json",
        "@drake//lcmtypes:schunk",
        "@gflags//:gflags",
        "@lcm//:lcm",
    ]
)
