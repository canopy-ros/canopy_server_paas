package main

import (
    "log"
    "github.com/docker/engine-api/client"
    "github.com/docker/engine-api/types"
    "golang.org/x/net/context"
    "time"
    "flag"
    "strings"
    "gopkg.in/mgo.v2"
)

type containerdb interface {
    init()
    containerlist() []string
    containerstatus(c string) string
    setprocesslist(c string, proclist string)
}

type hub struct {
    cli *client.Client
    cdb *containerdb
    containers map[string]*Container
}

var addr = flag.String("addr", "localhost", "http service addr")
var port = flag.String("port", "5000", "http service port")

func main() {
    flag.Parse()
    log.Println("Canopy container management server started.")
    // Start docker client
    defaultHeaders := map[string]string{"User-Agent": "engine-api-cli-1.0"}
    cli, err := client.NewClient("unix:///var/run/docker.sock",
        "v1.18", nil, defaultHeaders)
    if err != nil {
        panic(err)
    }
    cdb := redisdb{dbw: nil}
    cdb.init()
    h := hub{cli: cli, cdb: &cdb, containers: make(map[string]*Container)}
    // Connect to MongoDB database
    session, err := mgo.Dial(":3001")
    defer session.Close()
    // List all existing containers and create container objects for them
    options := types.ContainerListOptions{All: true}
    containers, err := cli.ContainerList(context.Background(), options)
    for _, cont := range containers {
        if len(cont.Names) == 0 {
            continue;
        }
        log.Println("Found container:", cont.Names[0][1:])
        h.containers[cont.Names[0][1:]] = &Container{name: cont.Names[0][1:], id: cont.ID, started: false, h: &h, quit: make(chan int), mgoSession: session}
        go h.containers[cont.Names[0][1:]].statusUpdater()
    }
    // Set running containers as running in their corresponding container objects
    options = types.ContainerListOptions{All: false}
    containers, err = cli.ContainerList(context.Background(), options)
    for _, cont := range containers {
        log.Println("Container already started:", cont.Names[0][1:])
        h.containers[cont.Names[0][1:]].started = true
    }
    for {
        // Get the containers that should be running
        list := cdb.containerlist()
        for _, cont := range list {
            split := strings.Split(cont, ":")
            cont = split[1]
            status := cdb.containerstatus(cont)
            if val, ok := h.containers[cont]; ok { // If container exists, start it
                if status == "running" {
                    if !val.started {
                        val.start()
                    }
                } else if status == "stopped" {
                    if val.started {
                        val.stop();
                    }
                }
            } else { // If container does not exist, create it and start it
                h.containers[cont] = create(cli, cont, &h, session)
                go h.containers[cont].statusUpdater()
                if status == "running" {
                    h.containers[cont].start()
                }
            }
        }
        // Synchronize containers and container object statuses
        options = types.ContainerListOptions{All: true}
        containers, err = cli.ContainerList(context.Background(), options)
        for _, cont := range containers {
            if len(cont.Names) == 0 {
                continue;
            }
            // Check if container is supposed to be running
            status := cdb.containerstatus(cont.Names[0][1:])
            if status == "" { // Container removed
                if h.containers[cont.Names[0][1:]].started {
                    h.containers[cont.Names[0][1:]].stop()
                }
                h.containers[cont.Names[0][1:]].remove()
                delete(h.containers, cont.Names[0][1:])
            } else {
                if cont.Status == "" || cont.Status[:6] == "Exited" {
                    if status == "running" { // If it is not running but supposed to be, start it
                        h.containers[cont.Names[0][1:]].start()
                    } else { // Update container object status
                        h.containers[cont.Names[0][1:]].started = false
                    }
                } else if cont.Status[:2] == "Up" {
                    if status == "stopped" { // If it is running but not supposed to be, stop it
                        h.containers[cont.Names[0][1:]].stop()
                    } else { // Update container object status
                        h.containers[cont.Names[0][1:]].started = true
                    }
                }
            }
        }
        time.Sleep(100 * time.Millisecond)
    }
}
