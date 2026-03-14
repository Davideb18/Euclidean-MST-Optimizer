#!/bin/bash

# Euclidean MST Optimizer Benchmark Script

# 1. Setup
mkdir -p bin
javac -d bin src/com/emst/optimizer/*.java

# 2. Generate large test dataset if not exists (1M points)
if [ ! -f "dataset/large_test.txt" ]; then
    echo "Generating 1,000,000 points..."
    python3 -c "import random; [print(f'({random.randint(0, 1000000)},{random.randint(0, 1000000)})') for _ in range(1000000)]" > dataset/large_test.txt
fi

# 3. Run Benchmark
echo "------------------------------------------------"
echo "Running EMST Optimizer on 1,000,000 points..."
echo "Alpha: 500"
echo "------------------------------------------------"

time java -cp bin com.emst.optimizer.Main dataset/large_test.txt 500

echo "------------------------------------------------"
echo "Benchmark completed."
