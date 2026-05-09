# Hybrid A* Planner

Hybrid A* planner на C++ для планирования траектории автономного автомобиля в 2D occupancy grid, с Python-визуализацией, сценариями, тестами и benchmark'ами.

Проект строится вокруг C++ core-библиотеки планировщика, явных форматов данных для сценариев/результатов и лёгких Python-утилит для визуализации и анализа benchmark'ов.

## Roadmap

1. `[готово]` Bootstrap workspace: структура репозитория, CMake, GoogleTest, минимальный executable, scenario-заглушки.
2. `[готово]` Common utilities: `Pose2D`, математические helpers, нормализация углов и unit tests.
3. `[готово]` Occupancy grid basics: размеры карты, resolution, occupied/free cells, bounds checking, world/grid преобразования.
4. `[готово]` Vehicle model basics: параметры автомобиля и простой kinematic bicycle step.
5. `[готово]` Vehicle footprint: oriented rectangle автомобиля по `Pose2D` и `VehicleParams`.
6. `[готово]` Collision checking: проверка footprint against occupancy grid.
7. `[готово]` Motion primitives: forward/reverse successors на основе vehicle model.
8. `[следующий этап]` Heuristic и path/result export.
9. `[в плане]` Baseline Hybrid A* search.
10. `[в плане]` Python plotting, animation scripts и benchmark statistics.
11. `[в плане]` Smoothing и coarse-to-fine refinement.

## Структура Репозитория

```text
.
├── configs/        # Конфигурации planner'а и benchmark'ов
├── docs/           # Заметки по дизайну и алгоритмам
├── include/        # Публичные C++ headers
├── src/            # Реализация C++ и entrypoint приложения
├── tests/          # Unit tests
├── scenarios/      # Входные JSON-сценарии
├── scripts/        # Python-утилиты для визуализации и benchmark'ов
├── results/        # Сгенерированные результаты planner'а
└── third_party/    # Опциональные внешние зависимости или заметки
```

## Сборка

Первый запуск CMake может скачать GoogleTest через `FetchContent`.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
ctest --test-dir build --output-on-failure
./build/hybrid_astar_app
```

## Текущий Статус

- C++ project scaffold готов.
- Common math/angle utilities покрыты unit tests.
- Occupancy grid реализует базовую карту и преобразования world/grid.
- Vehicle model реализует базовый kinematic bicycle step.
- Vehicle footprint вычисляет углы прямоугольного footprint автомобиля.
- `hybrid_astar_app` собирается как минимальное приложение.
- `hybrid_astar_tests` содержит GoogleTest-based unit tests; сейчас проходит `32/32` теста.
- Scenario JSON файлы пока являются заглушками и ещё не загружаются из C++.
