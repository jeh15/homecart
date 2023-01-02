# BUILD
package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "json_test",
    srcs = ["json_test.cc",],
    linkstatic = True,
    linkopts = ["-lpthread"],
    deps = [
        "@nlohmann_json//:json",
        "@ur_rtde",
        "@gflags//:gflags",
    ]
)