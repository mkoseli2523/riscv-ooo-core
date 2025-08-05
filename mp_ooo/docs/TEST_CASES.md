# SP 25 TEST CASES

## Open Test Cases (CP3/ Advanced Features Grading)

### Coremark
* Industry standard benchmarking suite

### FFT
* Performs Fast Fourier Transform on multiple input signals
* Utilizes arithmetic units heavily
* Recursive control flow

### Mergesort
* Performs merge sort on multiple input arrays
* Many data-dependent branches
* Many loads/stores
* Recursive control flow
 
### AES-SHA
* Performs AES encryptions and SHA-256 hash on input stream
* Has data reuse
* Many helper functions
* Utilizes arithmetic units heavily
* Expresses good ILP
* Many loads/stores, with high memory-level parallelism
  
### Compression
* Performs Huffman Compression on a random string from Mary Shelley's Frankenstein
* Low data reuse
* Large loops with data-dependent accesses. 
* Link to full text of Frankenstein: https://www.gutenberg.org/ebooks/84

## Closed Test Cases (Only For Competition)

### CNN
* Runs Convolutional Neural Network layers, including Conv2d, ReLU, and MaxPool
* Has high arithmetic intensity, predictable for loops
* Has a predictable memory access pattern

### Sudoku
* Recursively solves Sudoku board
* Has deep recursion
* Has branch correlation
* Has high memory reuse

### Graph
* Traverses linked structure and performs arithmetic operations on nodes
* Arithmetic portions have dependency chains
* Locations are loaded from memory in random order

### Physics
* Performs Matrix-Multiplication for mesh transformations, and uses Gilbert-Johnson-Keerthi algorithm for collision detection
* Heavy on arithmetic instructions
* Many helper function calls

### Ray Tracing
* Performs ray tracing based render of a simple scene with no reflections
* Expresses good ILP
* Many loads/stores, with high memory-level parallelism
