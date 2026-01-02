load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def _non_module_deps_impl(ctx):
    # MuJoCo binary release
    http_archive(
        name = "mujoco",
        urls = ["https://github.com/google-deepmind/mujoco/releases/download/3.3.4/mujoco-3.3.4-linux-x86_64.tar.gz"],
        sha256 = "ecf1a17459a342badf2b4f32dd4677a6a0e5fd393c5143993eb3e81b8e44609b",
        strip_prefix = "mujoco-3.3.4",
        build_file_content = """
package(default_visibility = ["//visibility:public"])
cc_library(
    name = "mujoco",
    hdrs = glob(["include/**/*.h"]),
    includes = ["include"],
    srcs = ["lib/libmujoco.so.3.3.4"],
    linkopts = ["-Wl,-rpath,lib"], # Optional help
)
""",
    )

non_module_deps = module_extension(
    implementation = _non_module_deps_impl,
)
