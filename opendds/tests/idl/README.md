To generate the code from the IDL use the following commands:

Make sure DDS_ROOT points to the correct directory, i.e. the cmake build dir.

```
export DDS_ROOT=/path/to/DDS_INSTALL_DIR/share/dds

$DDS_ROOT/bin/opendds_idl DDSPerfTest.idl
$DDS_ROOT/../../bin/tao_idl -I$DDS_ROOT/../../include -I$DDS_ROOT/../../include/orbsvcs DDSPerfTest.idl
$DDS_ROOT/../../bin/tao_idl -I$DDS_ROOT/../../include -I$DDS_ROOT/../../include/orbsvcs DDSPerfTestTypeSupport.idl
```


if you get the error ` line 33: Illegal syntax following module '{' opener`

read this:


It turns out that my DDS_ROOT was set to an incorrect path, but it was quite tricky to find, as at first sight the tooling seemed to work.

I will explain the steps I followed, together with the solution, so it may be of use to other readers.

- Downloaded the tarball
- ./configure --ace=download --prefix=/my/install/dir/OpenDDS
-  make && make install

Now, DO NOT set DDS_ROOT to /my/install/dir/OpenDDS, but instead point it to /my/install/dir/OpenDDS/share/dds. The reason for the empty module is that setting the wrong DDS_ROOT will prevent opendds_idl from finding IDLTemplate.txt, needed to create the Type Support.

I've checked the docs, but I'm not sure if this subtle difference regarding DDS_ROOT is stated somewhere explicitly.