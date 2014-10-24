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

## doxygen documentation generator (WORK IN PROGRESS)

Our plan is to document out classes and use doxygen to extract and create 
documentation for developers of the class objects and methods.

## other tools

When or if we add additional tools they will get documented here.


