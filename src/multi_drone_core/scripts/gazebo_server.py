#!/usr/bin/env python3

from typing import List
import subprocess
import os
import shutil


DEFAULT_DOWNLOAD_DIR = "https://github.com/PX4/PX4-gazebo-models/archive/refs/heads/main.zip"


def launch_process(cmd, env=None):
    process_env = os.environ.copy()
    if env:
        process_env.update(env)
    return subprocess.Popen(cmd, cwd='.', env=process_env)


def build_gazebo_command(
    world: str = "default",
    gz_partition: str = None,
    gz_ip: str = None,
    model_download_source: str = DEFAULT_DOWNLOAD_DIR,
    px4_model_store: str = "~/simulation-gazebo",
    custom_model_store: str = None,
    custom_model_store_other: List[str] = [],
    overwrite: bool = False,
    headless: bool = False,
):
    """
    Функция для настройки симуляции Gazebo с поддержкой кастомных моделей и миров.

    Args:
        world (str): Имя файла мира для симуляции (без расширения).
        gz_partition (str): Разделение ресурсов Gazebo (опционально).
        gz_ip (str): IP-адрес для сетевого интерфейса (опционально).
        model_download_source (str): URL или путь для загрузки моделей.
        px4_model_store (str): Локальная директория для хранения моделей PX4.
        custom_model_store (str): Директория для кастомных моделей.
        custom_model_store_other (List[str]): Дополнительные директории для моделей.
        overwrite (bool): Флаг, указывающий на перезапись существующих моделей.
        headless (bool): Флаг для запуска Gazebo без графического интерфейса.

    Returns:
        tuple[list[str], dict[str, str]]: Команда запуска Gazebo и переменные окружения.

    Example:
        >>> cmd, env = build_gazebo_command(world="test_world", headless=True)
        >>> gazebo_server = launch_process(cmd, env)
    """
    px4_model_store = os.path.expanduser(px4_model_store)

    if not os.path.exists(px4_model_store):
        print("Создаём директорию для хранения моделей...")
        os.makedirs(px4_model_store)

    model_count = int(subprocess.check_output(f'find {px4_model_store} -type f | wc -l', shell=True, text=True))
    models_exist = model_count > 0
    print(f"Обнаружено {model_count} файлов в директории {px4_model_store}")

    if models_exist and not overwrite:
        print("Директория с моделями не пуста, а параметр перезаписи не установлен. Пропускаем загрузку моделей.")
    elif overwrite and models_exist:
        try:
            subdirectories = [os.path.join(px4_model_store, d) for d in os.listdir(px4_model_store) if os.path.isdir(os.path.join(px4_model_store, d))]
            for directory in subdirectories:
                shutil.rmtree(directory)
            print("Режим перезаписи включён. Существующие поддиректории удалены.")
        except Exception as e:
            print(f"Ошибка при удалении: {e}")

    if not models_exist or overwrite:
        print("Загружаем модели из стандартного источника...")
        os.system(f'curl -L -o {px4_model_store}/resources.zip {model_download_source}')

        try:
            shutil.unpack_archive(f'{px4_model_store}/resources.zip', px4_model_store, 'zip')
        except Exception as e:
            print(f"Предупреждение: Не удалось распаковать модели из {px4_model_store}/resources.zip. Ошибка: {e}")

        os.system(f'mv {px4_model_store}/PX4-gazebo-models-main/models {px4_model_store}/models')
        os.system(f'mv {px4_model_store}/PX4-gazebo-models-main/worlds {px4_model_store}/worlds')
        os.system(f'rm {px4_model_store}/resources.zip')
        os.system(f'rm -rf {px4_model_store}/PX4-gazebo-models-main/')

    print(f'> Подготовлена симуляция Gazebo {world}')

    custom_model_paths = []
    world_path = f"{world}.sdf"

    if custom_model_store:
        custom_model_paths = [
            f"{custom_model_store}/models",
            f"{custom_model_store}/worlds"
        ]
        # world_path = f"{custom_model_store}/worlds/{world}.sdf"

    custom_model_paths.extend([f"{path}/models:{path}/worlds" for path in custom_model_store_other])

    gz_sim_resource_path = ':'.join(
        [f"{px4_model_store}/models", f"{px4_model_store}/worlds"] + custom_model_paths
    )

    print(f"Установлен GZ_SIM_RESOURCE_PATH: {gz_sim_resource_path}")

    gz_cmd = ['gz', 'sim', '-r', world_path]
    if headless:
        gz_cmd.append('-s')

    gz_env = {'GZ_SIM_RESOURCE_PATH': gz_sim_resource_path}
    if gz_partition:
        gz_env['GZ_PARTITION'] = gz_partition
    if gz_ip:
        gz_env['GZ_IP'] = gz_ip

    return gz_cmd, gz_env


def start_gazebo_simulation(
    world: str = "default",
    gz_partition: str = None,
    gz_ip: str = None,
    model_download_source: str = DEFAULT_DOWNLOAD_DIR,
    px4_model_store: str = "~/simulation-gazebo",
    custom_model_store: str = None,
    custom_model_store_other: List[str] = [],
    overwrite: bool = False,
    headless: bool = False,
):
    cmd, env = build_gazebo_command(
        world=world,
        gz_partition=gz_partition,
        gz_ip=gz_ip,
        model_download_source=model_download_source,
        px4_model_store=px4_model_store,
        custom_model_store=custom_model_store,
        custom_model_store_other=custom_model_store_other,
        overwrite=overwrite,
        headless=headless,
    )
    return launch_process(cmd, env)


if __name__ == "__main__":
    start_gazebo_simulation(world="walls")
