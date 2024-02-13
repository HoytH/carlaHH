How to start project
from the build folder run 
```cmake -S ../ -B . -DCMAKE_EXPORT_COMPILE_COMMANDS=1```

-DCMAKE_EXPORT_COMPILE_COMMANDS=1 is to let clangd know what libraries exist for auto complete
