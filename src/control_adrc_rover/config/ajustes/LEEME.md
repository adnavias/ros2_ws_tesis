# Ajustes guardados por plataforma

Aquí el control guarda `<brazo>_<plataforma>.yaml` (por ejemplo
`principal_sim.yaml` y `principal_real.yaml`) cuando se usa
"Guardar ajustes" o "Guardar plataforma" en la interfaz, o al terminar
el autotuning con "Guardar al YAML".

Al arrancar se aplican ENCIMA de `config/brazos/<brazo>.yaml` y
`config/plataformas/<plataforma>.yaml`. Si borras el archivo, el control
arranca con los valores de esos dos (los "de fábrica").

Con `colcon build --symlink-install` se escriben aquí, en `src/`, y git
los ve. Sin `--symlink-install` quedan en `install/` y se pierden al
recompilar.
