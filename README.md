# Hexagonal Grid Pathfinding (Movhex)

Implementation of an optimized pathfinding engine for a logistic company operating on a hexagonal grid map. The system handles dynamic edge weights (air routes) and terrain costs.

## Key Features

* **Algorithm:** Dijkstra's Algorithm adapted for hexagonal topology using cube coordinates conversion.
* **Optimization:** Implemented a custom Hash Map caching system to handle repetitive queries efficiently (`O(1)` lookup for cached routes).
* **Performance:** Written in **pure C** using manual memory management and flat arrays for cache locality.

## Context
Developed as a final project for the *Algorithms and Data Structures* course at **Politecnico di Milano**.
