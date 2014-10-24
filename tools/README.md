# vrp-tools

These are some general purpose project tools for vehicle-routing-problems.
They can be invoke using the make.

```
make            # report the various options available
make astyle     # runs astyle source formater over our code
make doxygen    # will eventually run doxygen documentation generator
```

## astyle source code formatter

We are using astyle version 2.04 and the project config file is ``astyle.conf``
and we also astyle-2.01.conf file for those that need an older version.
Basically you should not need to use this. This is an internal tool and we will
apply it to the source before releases or otherwise as appropriate.

For more information see also: http://astyle.sourceforge.net/

A typical use case for this would be to create a branch off of ``develop``, then
run the formatter, inspect the code, if it is ok, then merge it back into
``develop`` and push it. For example:

```
cd vehicle-routing-problems/tools
git checkout -b test-astyle develop
make astyle
cd ..
# inspect the source files

# if you are happy with the formating
git commit -a -m "Running astyle to format the source"
git checkout develop
git merge --no-ff test-astyle
git push

# if you were just curious or want to abandon this then

# commit to clean up so we can get out of here
git commit -a -m "Running astyle to format the source"
# switch to another branch
git checkout develop
# delete out local test-astyle branch that we don't want
git branch -D test-astyle
```

Developers should NOT reformat and push code. Only the system integrator
that does the project merges should do this AND only when everyone has pushed
all the work in progress and it has been merged into ``develop`` first or it
will cause LOTS of conflicts that will need to be resolved.

## doxygen documentation generator (WORK IN PROGRESS)

Our plan is to document out classes and use doxygen to extract and create 
documentation for developers of the class objects and methods.

## other tools

When or if we add additional tools they will get documented here.


