# BUILD
cc_binary(
    name = "save_home_position",
    srcs = ["save_home_position.cc",],
    linkstatic = True,
    linkopts = ["-lpthread"],
    deps = [
        "@nlohmann_json//:json",
        "@ur_rtde",
        "@gflags//:gflags",
    ]
)

cc_binary(
    name = "go_to_home_position",
    srcs = ["go_to_home_position.cc",],
    linkstatic = True,
    linkopts = ["-lpthread"],
    deps = [
        "@nlohmann_json//:json",
        "@ur_rtde",
        "@gflags//:gflags",
    ]
)