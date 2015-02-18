project(bullet_narrowphase)
ADD_EXECUTABLE(bullet_narrowphase	main.cpp narrowphase/gjk_epa.cpp narrowphase/voronoi_simplex.cpp)
