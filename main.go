package main

import (
    "log"
    "github.com/garyburd/redigo/redis"
    "github.com/docker/engine-api/client"
    "github.com/docker/engine-api/types"
    "golang.org/x/net/context"
    "time"
    "flag"
    "strings"
)

type hub struct {
    cli *client.Client
    dbw *dbwriter
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
    // Connect to redis database
    c, err := redis.Dial("tcp", ":6379")
    if err != nil {
        panic(err)
    }
    dbw := dbwriter{redisconn: &c, commChannel: make(chan command, 2)}
    go dbw.writer()
    h := hub{cli: cli, dbw: &dbw, containers: make(map[string]*Container)}
    // List all existing containers and create container objects for them
    options := types.ContainerListOptions{All: true}
    containers, err := cli.ContainerList(context.Background(), options)
    for _, cont := range containers {
        log.Println("Found container:", cont.Names[0][1:])
        h.containers[cont.Names[0][1:]] = &Container{name: cont.Names[0][1:], id: cont.ID, started: false, h: &h, quit: make(chan int)}
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
        reply, _ := redis.Values(dbw.write("SCAN", 0, "match", "containers:*:status"))
        var temp int
        var list []string
        reply, _ = redis.Scan(reply, &temp, &list)
        for _, cont := range list {
            split := strings.Split(cont, ":")
            status, _ := redis.String(dbw.write("GET", cont))
            cont = split[1];
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
                h.containers[cont] = create(cli, cont, &h)
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
            // Check if container is supposed to be running
            status, _ := redis.String(dbw.write("GET", "containers:" + cont.Names[0][1:] + ":status"))
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
