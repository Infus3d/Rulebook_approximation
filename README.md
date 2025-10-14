# A*pex
C++ implementations of bi- or multi- objective search algorithms like BOA\*, PPA\* and A\*pex.


You can compile the project using the following commands:

``` shell
cmake .
make
```
After typing the above commands. Cmake will generate `multiobj` in the `bin` folder.
You can type `multiobj --help` to see the expected input arguments.

Example usage:

``` shell
# find the Pareto-optimal frontier from node 20002 to node 164983 in the BAY map.
./bin/multiobj -m resources/dataset/USA-road-d.BAY.gr resources/dataset/USA-road-t.BAY.gr -s 20002 -g 164983 -a Apex -o output.txt
```

You can download the road networks we used in the paper from [here]( http://users.diag.uniroma1.it/challenge9/download.shtml).
Files for the added third objectives and the testing cases used in our paper could be found [here](https://drive.google.com/drive/folders/10o91HGS6KCtbndX23vn1-gL7a_2aWxHt?usp=sharing).
