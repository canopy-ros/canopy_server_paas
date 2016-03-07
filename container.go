package main

import (
    "log"
    "time"
    "github.com/docker/engine-api/types/container"
    "github.com/docker/engine-api/types/network"
    "github.com/docker/engine-api/client"
    "github.com/docker/engine-api/types"
    "strings"
    "encoding/json"
)

type Container struct {
    name string
    id string
    started bool
    h *hub
    quit chan int
}

type Process struct {
    Name string `json:"name,omitempty"`
    User string `json:"user,omitempty"`
    PID string `json:"pid,omitempty"`
    STime string `json:"stime,omitempty"`
    Time string `json:"time,omitempty"`
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
        Image:"canopy",
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
    return &Container{name: name, id: id.ID, started: false, h: h, quit: make(chan int)}
}

func (c *Container) statusUpdater() {
    for {
        exit := false
        select {
            case <- c.quit:
                exit = true
            default:
            list, err := c.h.cli.ContainerTop(c.id, []string{})
            if err == nil {
                processes := make([]Process, 0)
                for _, p := range list.Processes {
                    proc := Process{Name: p[7], User: p[0], PID: p[1], STime: p[4], Time: p[6]}
                    processes = append(processes, proc)
                }
                res, _ := json.Marshal(processes)
                c.h.dbw.write("SET", "containers:" + c.name + ":processes", res)
            } else {
                c.h.dbw.write("SET", "containers:" + c.name + ":processes", "[]")
        }
        // read, err := c.h.cli.ContainerStats(context.TODO(), c.id, false)
        // if err == nil {
        //     out := make([]byte, 1788)
        //     read.Read(out)
        //     read.Close()
        // }
        if exit {
            break;
        }
        time.Sleep(100 * time.Millisecond)
        }
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

func (c *Container) remove() {
    containerRmOptions := types.ContainerRemoveOptions{
        ContainerID: c.id,
        RemoveVolumes: true,
        Force: true,
    }
    c.h.cli.ContainerRemove(containerRmOptions)
    c.started = false
    close(c.quit)
    log.Println("Removed container:", c.name)
}
