# Hexagonal Grid Pathfinding (Movhex)

An optimized pathfinding engine in C for a logistics company operating on a hexagonal grid map. The system handles dynamic edge weights (air routes) and terrain costs.

## Key features

- **Algorithm:** Dijkstra's algorithm adapted to hexagonal topology using cube coordinates conversion.
- **Optimization:** a custom hash map cache for repeated queries, with `O(1)` lookup of cached routes.
- **Performance:** pure C with manual memory management and flat arrays for cache locality.

## Build

```
gcc -O2 -Wall -o hex_pathfinding hex_pathfinding.c -lm
```

## Usage

The program reads commands from standard input:

| Command | Arguments | Effect |
| --- | --- | --- |
| `init` | `columns rows` | Creates the map |
| `change_cost` | `x y v radius` | Changes the terrain cost around a cell |
| `toggle_air_route` | `x1 y1 x2 y2` | Adds or removes an air route between two cells |
| `travel_cost` | `xs ys xd yd` | Prints the minimum travel cost between two cells |
| `quit` | | Exits |

## Context

Final project for the *Algorithms and Data Structures* course at Politecnico di Milano.
