package main

import (
    "log"
    "time"
    "github.com/docker/engine-api/types/container"
    "github.com/docker/engine-api/types/network"
    "github.com/docker/engine-api/client"
    "github.com/docker/engine-api/types"
    "golang.org/x/net/context"
    "strings"
    "encoding/json"
    "gopkg.in/mgo.v2"
    "gopkg.in/mgo.v2/bson"
)

type Container struct {
    name string
    id string
    started bool
    h *hub
    quit chan int
    mgoSession *mgo.Session
}

type Process struct {
    Name string `json:"name,omitempty"`
    User string `json:"user,omitempty"`
    PID string `json:"pid,omitempty"`
    STime string `json:"stime,omitempty"`
    Time string `json:"time,omitempty"`
}

type MgoURL struct {
    Container string `bson:"container,omitempty"`
    User string `bson:"user,omitempty"`
    Url string `bson:"url,omitempty"`
}

func create(cli *client.Client, name string, h *hub, session *mgo.Session) *Container {
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
    id, err := cli.ContainerCreate(context.Background(), &containerOptions, &hostOptions, &network.NetworkingConfig{}, name)
    if err != nil {
        log.Println(err)
        return nil
    }
    log.Println("Created container:", name)
    return &Container{name: name, id: id.ID, started: false, h: h, quit: make(chan int), mgoSession: session}
}

func (c *Container) statusUpdater() {
    for {
        exit := false
        select {
            case <- c.quit:
                exit = true
            default:
            list, err := c.h.cli.ContainerTop(context.Background(), c.id, []string{})
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
    col := c.mgoSession.DB("meteor").C("urls")
    user := c.name[:strings.Index(c.name, "_")]
    name := c.name[strings.Index(c.name, "_") + 1:]
    result := MgoURL{}
    e := col.Find(bson.M{"container": name, "user": user}).One(&result)
    log.Println(result.Url)
    if e != nil {
        log.Fatal(e)
    }
    err := c.h.cli.ContainerStart(context.Background(), c.id)
    if err != nil {
        return
    }
    c.started = true
    log.Println("Started container:", c.name)
}

func (c *Container) stop() {
    c.h.cli.ContainerStop(context.Background(), c.id, 10)
    c.started = false
    log.Println("Stopped container:", c.name)
}

func (c *Container) remove() {
    containerRmOptions := types.ContainerRemoveOptions{
        RemoveVolumes: true,
        Force: true,
    }
    c.h.cli.ContainerRemove(context.Background(), c.id, containerRmOptions)
    c.started = false
    close(c.quit)
    log.Println("Removed container:", c.name)
}
