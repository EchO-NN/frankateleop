from polymetis import FrankaInterfaceServer
import zerorpc


if __name__ == "__main__":
    server = FrankaInterfaceServer()
    s = zerorpc.Server(server)
    s.bind("tcp://0.0.0.0:4242")
    s.run()