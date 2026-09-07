# sf_app_hooks — L1 application entry point (Topic API), see app_hooks.hpp.
#
# SF_APP_DIR (a cache variable, e.g. `idf.py -D SF_APP_DIR=/abs/path build`)
# selects an application directory whose *.cpp provide sf::app::* instead of
# app_default.cpp. Those application sources are compiled straight into the
# main component (see ../../main/CMakeLists.txt), NOT into this component, so
# that they share the main component's REQUIRES without a component-level
# dependency edge back to it (a component cannot REQUIRES its own consumer).
# This component then becomes header-only — it still declares the contract
# (app_hooks.hpp) that both app_default.cpp and the user's *.cpp implement.
#
# sf_app_hooks — L1（Topic API）アプリの入口。詳細は app_hooks.hpp 参照。
#
# SF_APP_DIR（キャッシュ変数。例: `idf.py -D SF_APP_DIR=/abs/path build`）が
# 与えられたら、そのディレクトリの *.cpp が app_default.cpp の代わりに
# sf::app::* を実装する。それらアプリソースは本コンポーネントではなく main
# コンポーネントへ直接コンパイルする（../../main/CMakeLists.txt 参照）—
# main の REQUIRES をそのまま共有でき、コンポーネントが自分の利用者へ逆向きの
# 依存辺を持たずに済む。本コンポーネントはこのときヘッダのみになる —
# それでも app_default.cpp とユーザーの *.cpp が実装する契約
# （app_hooks.hpp）は引き続き本コンポーネントが宣言する。
if(SF_APP_DIR)
    idf_component_register(
        INCLUDE_DIRS "include"
        REQUIRES sf_controller sf_estimator
    )
else()
    idf_component_register(
        SRCS "app_default.cpp"
        INCLUDE_DIRS "include"
        REQUIRES sf_controller sf_estimator
        PRIV_REQUIRES
            sf_controller_pid
            sf_estimator_eskf
            sf_estimator_complementary
            sf_core
    )
endif()
