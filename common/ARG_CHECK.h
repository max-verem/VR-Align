#define ARG_CHECK_BEGIN(PARAM_NAME, PARAMS_CNT) if (!strcmp("--" PARAM_NAME, argv[i]) && (i + PARAMS_CNT) < argc) {
#define ARG_CHECK_END(PARAMS_CNT) i += PARAMS_CNT + 1; }

