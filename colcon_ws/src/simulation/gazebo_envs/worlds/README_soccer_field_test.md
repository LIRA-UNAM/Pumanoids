# soccer_field_test.world - Depuración

Mundo mínimo para encontrar qué modelo causa que el robot no aparezca.

## Cómo usar

1. **Compilar**:
   ```bash
   cd colcon_ws
   colcon build --packages-select gazebo_envs surge_et_ambula
   source install/setup.bash
   ```

2. **Lanzar el test** (con spawn automático):
   ```bash
   ros2 launch surge_et_ambula g1_sim_humble_test.launch.py
   ```

3. **Ir descomentando** los bloques `<include>` en `soccer_field_test.world`:
   - Primero probar sin ningún include (solo ground_plane inline)
   - Si el robot aparece → descomentar `soft_ground_plane`
   - Si sigue bien → descomentar `teensize_field`
   - Luego `teensize_ball`, `teensize_goal`, `teensize_goal2`
   - Cuando deje de aparecer el robot, ese modelo es el problemático

4. **Recompilar** después de cada cambio al .world:
   ```bash
   colcon build --packages-select gazebo_envs
   source install/setup.bash
   ```

## Ejecutar create por separado (ver errores)

Para ver el error que arroja el nodo `create`:

**Terminal 1** - Lanzar simulación SIN spawn del robot:
```bash
source install/setup.bash
ros2 launch surge_et_ambula g1_sim_humble_test.launch.py spawn_robot:=false
```

Esperar a que Gazebo cargue completamente (~10 s).

**Terminal 2** - Ejecutar create manualmente:
```bash
source install/setup.bash
ros2 run ros_gz_sim create -world default -topic /robot_description -name G1 -x 0 -y 0 -z 1.5
```

Los mensajes de error aparecerán en la Terminal 2.
