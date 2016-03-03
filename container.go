package main

import (
    "log"
    "time"
    "github.com/docker/engine-api/types/container"
    "github.com/docker/engine-api/types/network"
    "github.com/docker/engine-api/client"
    "strings"
)

type Container struct {
    name string
    id string
    started bool
    h *hub
}

func create(cli *client.Client, name string, h *hub) *Container {
    split := strings.SplitN(name, "_", 2)
    containerOptions := container.Config{
        Hostname:name,
        User:"",
        AttachStdin:true,
        AttachStdout:true,
        AttachStderr:true,
        Tty:true,
        OpenStdin:true,
        StdinOnce:true,
        Image:"roscloud",
        WorkingDir:"",
        Env:[]string{"HOST=" + *addr, "PORT=" + *port, "KEY=" + split[0], "NAME=" + split[1]},
    }
    hostOptions := container.HostConfig{
        NetworkMode: "bridge",
    }
    id, err := cli.ContainerCreate(&containerOptions, &hostOptions, &network.NetworkingConfig{}, name)
    if err != nil {
        log.Println(err)
        return nil
    }
    log.Println("Created container:", name)
    return &Container{name: name, id: id.ID, started: false, h: h}
}

func (c *Container) statusUpdater() {
    for {
        list, err := c.h.cli.ContainerTop(c.id, []string{})
        if err == nil {
            c.h.dbw.write("HMSET", "containers:" + c.name, "processes", list.Processes)
        } else {
            c.h.dbw.write("HMSET", "containers:" + c.name, "processes", "[]")
        }
        // read, err := c.h.cli.ContainerStats(context.TODO(), c.id, false)
        // if err == nil {
        //     out := make([]byte, 1788)
        //     read.Read(out)
        //     read.Close()
        // }
        c.h.dbw.write("HMSET", "containers:" + c.name, "running", c.started)
        time.Sleep(100 * time.Millisecond)
    }
}

func (c *Container) start() {
    err := c.h.cli.ContainerStart(c.id)
    if err != nil {
        return
    }
    c.started = true
    log.Println("Started container:", c.name)
}

func (c *Container) stop() {
    c.h.cli.ContainerStop(c.id, 10)
    c.started = false
    log.Println("Stopped container:", c.name)
}
