package main

import (
    "github.com/garyburd/redigo/redis"
)

type redisdb struct {
    dbwriter *dbw
}

func (r redisdb) init() {
    // Connect to redis database
    c, err := redis.Dial("tcp", ":6379")
    if err != nil {
        panic(err)
    }
    r.dbw := dbwriter{redisconn: &c, commChannel: make(chan command, 2)}
    go r.dbw.writer()
}

func (r redisdb) containerlist() []string {
    iter := 0
    var list []string
    for {
        reply, _ := redis.Values(r.dbw.write("SCAN", iter, "match", "containers:*:status"))
        var temp int
        var templist []string
        reply, _ = redis.Scan(reply, &temp, &templist)
        iter = temp
        list = append(list, templist...)
        if iter == 0 {
            break
        }
    }
    return list
}

func (r redisdb) containerstatus(c string) string {
    status, _ := redis.String(r.dbw.write("GET", "containers:" + c + ":status"))
    return status
}

func (r redisdb) setprocesslist(c string, proclist string) {
    r.dbw.write("SET", "containers:" + c + ":processes", proclist)
}
