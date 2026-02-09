# RA*pex
C++ implementations of bi- or multi- objective search algorithms like BOA\*, PPA\*, A\*pex, Rulebook Planning, and RA\*pex.


You can compile the project using the following commands:

``` shell
cmake .
make
```
After typing the above commands. Cmake will generate `multiobj` in the `bin` folder.
You can type `multiobj --help` to see the expected input arguments.

#### Example usage:

``` shell
# find the optimal frontier for the start and goal queries given in the query file. Note that eps=0 should be set in the 'rules' file given by -r.
./bin/multiobj
-m
resources/dataset/USA-road-d.BAY.gr
resources/dataset/USA-road-t.BAY.gr
-a
RApex
-q
../resources/dataset/BAY_instances.txt
-r
../resources/dataset/3_rules_eps_0.txt
-a
RApex
-o
output.txt
```

``` shell
# find the approximate optimal frontier for the start and goal queries given in the query file.
./bin/multiobj
-m
../resources/dataset/USA-road-d.BAY.gr
../resources/dataset/USA-road-t.BAY.gr
../resources/dataset/USA-road-3.BAY.gr
-q
../resources/dataset/BAY_instances.txt
-r
../resources/dataset/3_rules_eps_0.01.txt
-a
RApex
-o
output.txt
```

#### Explanation of the rules file:  
The file you provide with the flag '-r rules_file.txt' needs to follow the outline below (remove the comments).
```c++
4                           // number of rules/objectives
0.01 0.01 0.01 0.01         // eps_i for each rule i
3                           // number of equivalence classes in the quotient set of the rules
2 0 1                       // Eq. class #1 contains 2 elements followed by the rules id 0 and 1
1 2                         // Eq. class #2 contains only 1 element followed by the rule id (2)
1 3                         // Eq. class #3 contains only 1 element followed by the rule id (3)
4                           // number of rule relationships
0 1                         // rule 0 is strictly better than rule 1
1 0                         // rule 1 is strictly better than rule 0
0 2                         // rule 0 is strictly better than rule 2
1 3                         // rule 1 is strictly better than rule 3
```

You can download the road networks we used in the paper from [here](https://www.diag.uniroma1.it/~challenge9/download.shtml).
Files for the added third and fourth (BAY only) objectives and the testing cases, including the rule files, used in our paper could be found [here](https://drive.google.com/drive/folders/12lVT4jmPKLb9zo8yB_NtLk7mrMwi-vKQ?usp=sharing).
