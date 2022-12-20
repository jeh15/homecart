# BUILD

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "json_test",
    srcs = ["json_test.cc",],
    linkstatic = True,
    deps = [
        "@nlohmann_json//:json",
        ],
    linkopts = [
        "-lpthread",
        ],
)