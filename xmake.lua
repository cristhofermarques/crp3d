add_rules("mode.debug", "mode.release")

cpp = "c++14"
binaries_directory = "binaries/"
rp3d_repository = "../reactphysics3d/"
rp3d_f64_define = "RP3D_DOUBLE_PRECISION_ENABLED"

option("rp3d_precision")
    set_showmenu(true)
    set_description("rp3d float precision")
    set_default("f32")
option_end()

if is_config("rp3d_precision", "f64") then
    add_defines(rp3d_f64_define)
end


function setup_runtime()
    if is_mode("debug") then
        set_runtimes("MTd")
    else
        set_runtimes("MT")
    end
end

function binary_postfix()
    local postfixes = ""
    if is_os("windows") then
        postfixes = postfixes .. "_windows"
    end

    if is_arch("x64") then
        postfixes = postfixes .. "_amd64"
    elseif is_arch("x86") then
        postfixes = postfixes .. "_i386"
    end

    if is_mode("debug") then
        postfixes = postfixes .. "_debug"
    elseif is_mode("release") then
        postfixes = postfixes .. "_release"
    end

    if is_config("rp3d_precision", "f64") then
        postfixes = postfixes .. "_f64"
    else
        postfixes = postfixes .. "_f32"
    end

    return postfixes
end

target("rp3d")
    set_kind("static")
    setup_runtime()
    set_languages(cpp)
    add_options("rp3d_precision")
    set_basename("rp3d" .. binary_postfix())
    set_targetdir(binaries_directory)
    add_includedirs(rp3d_repository .. "include")
    add_files(rp3d_repository .. "src/**.cpp")

target("crp3d")
    set_kind("static")
    setup_runtime()
    set_languages(cpp)
    add_options("rp3d_precision")
    set_basename("crp3d" .. binary_postfix())
    set_targetdir(binaries_directory)
    add_includedirs(rp3d_repository .. "include")
    add_files("crp3d.cpp")
    add_deps("rp3d")