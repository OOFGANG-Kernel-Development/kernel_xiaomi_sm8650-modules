load(":eva_modules.bzl", "eva_modules")
load(":eva_module_build.bzl", "define_consolidate_gki_modules")

def define_peridot():
    define_consolidate_gki_modules(
        target = "peridot",
        registry = eva_modules,
        modules = [
            "msm-eva",
        ],
        config_options = [
            "TARGET_SYNX_ENABLE",
            "TARGET_DSP_ENABLE",
            "CONFIG_EVA_PINEAPPLE",
            "CONFIG_MSM_MMRM"
        ],
    )
