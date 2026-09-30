
contains(CONFIG, robotkinematics_mesh_collision) {
    # COAL_ROOT, BOOST_ROOT, BOOST_INCLUDE_DIR and ASSIMP_ROOT are resolved by
    # qmake/dependency_paths.pri: qmake command line > environment > qmake/local_paths.pri.
    include($$PWD/qmake/dependency_paths.pri)
    isEmpty(MESH_COLLISION_BACKEND): MESH_COLLISION_BACKEND = $$(MESH_COLLISION_BACKEND)

    isEmpty(MESH_COLLISION_BACKEND) {
        error(MESH_COLLISION_BACKEND must be set when CONFIG+=robotkinematics_mesh_collision)
    }

    equals(MESH_COLLISION_BACKEND, coal) {
        isEmpty(COAL_ROOT) {
            error(COAL_ROOT must point to the Coal install root when MESH_COLLISION_BACKEND=coal. Set it in qmake/local_paths.pri.)
        }
        isEmpty(BOOST_ROOT) {
            error(BOOST_ROOT must point to the Boost install root when MESH_COLLISION_BACKEND=coal. Set it in qmake/local_paths.pri.)
        }
        isEmpty(BOOST_INCLUDE_DIR): BOOST_INCLUDE_DIR = $$BOOST_ROOT/include/boost-1_87

        DEFINES += ROBOTKINEMATICS_HAVE_COAL_MESH_BACKEND
        INCLUDEPATH += \
            $$COAL_ROOT/include \
            $$BOOST_INCLUDE_DIR

        !isEmpty(ASSIMP_ROOT) {
            INCLUDEPATH += $$ASSIMP_ROOT/include
            ROBOTKINEMATICS_RUNTIME_PATHS += $$ASSIMP_ROOT/bin
        }
        LIBS += -L$$COAL_ROOT/lib -lcoal

        ROBOTKINEMATICS_RUNTIME_PATHS += $$COAL_ROOT/bin $$BOOST_ROOT/lib
        include($$PWD/qmake/runtime_env.pri)
    } else {
        error(Unsupported MESH_COLLISION_BACKEND value: $$MESH_COLLISION_BACKEND)
    }
}
