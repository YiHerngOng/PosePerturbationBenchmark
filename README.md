Protocol of pose perturbation / hallucination benchmark
1. generate pose variation from ppB_Benchmark.py
2. read file 
3. pick an axis from xyz
4. check if extremes are tested
5. Test extremes and conduct binary search
6. Repeat 3 - 5 until all translational axes are found
7. Compute translational variation
8. randomly pick an axis from rpw
9. check if extremes are tested
10. Test extremes and conduct binary search
11. Repeat 8 - 10 until all translational axes are found
12. Compute orientation variation
13. Randomly pick a pose and start grasping