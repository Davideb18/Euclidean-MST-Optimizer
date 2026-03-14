# Euclidean Minimum Spanning Tree (EMST) Optimizer

[![Java](https://img.shields.io/badge/Language-Java-orange.svg)](https://www.java.com/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

A high-performance Java implementation designed to solve the **Euclidean Minimum Spanning Tree** problem for massive spatial datasets. By leveraging **Prim's Algorithm** combined with a specialized **spatial pruning technique** based on coordinate sorting, this optimizer achieves sub-second performance on datasets exceeding 1,000,000 points.

## 🚀 Key Features

*   **Massive Scalability**: Optimized for 1M+ coordinates in under 1 second.
*   **Spatial Pruning**: Implements a custom sorting-based pruning that reduces neighbor search from $O(N^2)$ to nearly $O(N \log N)$ for distributed spatial data.
*   **Memory Efficient**: Uses optimized data structures (Priority Queues, custom Point mappings) to minimize GC overhead.
*   **CLI Ready**: Easy-to-use interface for batch processing spatial datasets.

## 🧠 Technical Insight: How it works

Traditional MST approaches on complete graphs require $O(N^2)$ edges. In a Euclidean plane, many of these edges can be safely ignored.

1.  **Coordinate Sorting**: Points are initially sorted by their X-coordinate ($O(N \log N)$).
2.  **Distance-Based Pruning**: During Prim's expansion, the search for potential neighbors is constrained by a threshold $\alpha$.
3.  **Early Exit**: Because points are sorted by X, the optimizer stops searching for neighbors as soon as the horizontal distance $\Delta X$ exceeds $\alpha$, avoiding millions of unnecessary Euclidean distance calculations.

## 📊 Performance Benchmarks

| Dataset Size | Alpha ($\alpha$) | Execution Time (Avg) |
| :--- | :--- | :--- |
| 10,000 Points | 100 | ~0.05s |
| 100,000 Points | 250 | ~0.15s |
| **1,000,000 Points** | **500** | **~0.60s** |

*Benchmarks conducted on a modern JVM (Java 21) running on Apple M-series silicon.*

## 🛠️ Usage

### Build
Compile the source code using `javac`:
```bash
mkdir -p bin
javac -d bin src/com/emst/optimizer/*.java
```

### Run
Execute the solver by providing a dataset file and an $\alpha$ value:
```bash
java -cp bin com.emst.optimizer.Main dataset/large_test.txt 500
```

*Dataset format: One coordinate per line as `(x,y)`.*

## 📂 Project Structure
```text
├── src/com/emst/optimizer/
│   ├── Main.java       # CLI Entry point & I/O
│   ├── EMSTSolver.java # Core Prim's logic & pruning
│   ├── Point.java      # Spatial data structure
│   └── MSTNode.java    # Priority Queue node
└── bin/                # Compiled classes
```

---
*Created for the "Progetto Dati e Algoritmi" - University project.*
