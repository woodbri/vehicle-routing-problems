


## Tools

http://wiki.openstreetmap.org/wiki/Osmconvert



```

ipcs -l     ## To list Shared Memory Limits

ipcs -m     ## To list Shared Memory Segments

ipcs -m -i <shmid>  ## To display segment details

ipcrm shm <shmid>   ## To remove the shared memory segment.

# clean up and remove shared memory stuff
ls /dev/shm/ -l
sudo rm /dev/shm/*
ipcs -m
sudo ipcrm shm <shmid> <shmid> ...


## load osrm data into shared memory
NOTE it is required that you give the absolute path to data.osrm
otherwise shared memory processes running in other directories will not work

sudo -u <user> osrm-datastore /path/to/data.osrm

## run the routed using shared memory data
sudo -u <user> osrm-routed --sharedmemory=yes

sudo killall osrm-routed        ## kill the osrm-routed

sudo -u <user> osrm-datastore --sharedmemory=off    ## remove shared memory

```

# Configuring and using Shared Memory with OSRM

Note: This applies to version 0.3.7+ only.

Usually, when you start an application and it allocates some block of memory, this is free'd after your application terminates. And also, this memory is only accessible to a single process only. And then, there is _shared memory_. It allows you to share data among a number of processes and the shared memory we use is persistent. It stays in the system until it is explicitly removed.

By default, there is some restriction on the size of and shared memory segment. Depending on your distribution and its version it may be as little as 64 kilobytes. This is of course not enough for serious applications. 

The following gives a brief description on how to set the limits in a way that you (most probably) won't ever run into them in the future. Please read up on actual settings for your production environment in the manpage of shmctl, or consult further information, eg. [here](https://www.zabbix.org/wiki/How_to/configure_shared_memory).

First, we are going to raise the system limits. Second, we are going to raise the user limits.

System Limits
---

Append the following lines to `/etc/sysctl.conf`:

```Txt
kernel.shmall = 1152921504606846720
kernel.shmmax = 18446744073709551615
```

and then run `sysctl -p` with super-user privileges. Then check if settings were accepted:

```
$ ipcs -lm

------ Shared Memory Limits --------
max number of segments = 4096
max seg size (kbytes) = 18014398509481983
max total shared memory (kbytes) = 4611686018427386880
min seg size (bytes) = 1
```

User Limits
---

This is only half of the story. On Linux, only the super user is [allowed](https://git.kernel.org/cgit/linux/kernel/git/torvalds/linux.git/tree/include/uapi/linux/shm.h?id=refs/tags/v3.12#n52) to lock arbitrary amounts of shared memory into RAM. To fix this, we need to set the user limits properly. Let's have a look at what Ubuntu 12.10 sets by default:

```Bash
$ ulimit -a|grep max
max locked memory       (kbytes, -l) 64
max memory size         (kbytes, -m) unlimited
```

So, as a user we are allowed to only lock at most 64 KiB into RAM. This is obviously not enough. The settings can be changed by editing `/etc/security/limits.conf`. Add the following lines to the file, to raise the user limits to 64 GiB. At the time of writing, this is enough to do planet-wide car routing.

```
<user>           hard    memlock         unlimited
<user>           soft    memlock         68719476736
```

Note that <user> is the user name under which the routing process is running, and you need to re-login to activate these changes. If the user does not have a login, you can use `sudo -i -u <user>` to simulate an initial login. 

Note that on Ubuntu 12.04 LTS it is also necessary to edit `/etc/pam.d/su` (and `/etc/pam.d/common-session`) and remove the comment from the following line in order to activate `/etc/security/limits.conf`:

```Bash
session    required   pam_limits.so
```

Using Shared Memory
---

With all these changes done, you should now load all shared memory directly into RAM. Loading data into shared memory is as easy as 

```Bash
$ ./osrm-datastore /path/to/data.osrm
```

If there is insufficient available RAM (or not enough space configured), you will receive the following warning when loading data with `osrm-datastore`:

```Bash
[warning] could not lock shared memory to RAM
```

In this case, data will be swapped to a cache on disk, and you will still be able to run queries. But note that caching comes at the prices of disk latency.

Starting the routing process and pointing it to shared memory is also very, very easy:

```Bash
$ ./osrm-routed --sharedmemory=yes
```
