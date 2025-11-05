
package com.ecosense;

//default imported packages
import java.net.InetSocketAddress;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;

import com.ecosense.Utils.NetworkUtils;


public class mainServer extends WebSocketServer {

    private final Map<String, String> predefinedID = new ConcurrentHashMap<>();
    
    private final Map<String, WebSocket> idtosession = new ConcurrentHashMap<>();

    private final Map<WebSocket, WebSocket> SessionToPartner = new ConcurrentHashMap<>();


    public mainServer(InetSocketAddress address) {
        super(address);
        predefinedID.put("0001", "0002");
        predefinedID.put("0003", "0004");
    }

    @Override
    public void onStart() {
        System.out.println("Server is starting....");
    }

    @Override
    public void onOpen(WebSocket conn, ClientHandshake handshake) {
        System.out.println("connection by>" + conn.getRemoteSocketAddress());
    }

    @Override
    public void onMessage(WebSocket conn, String message) {


        //Important block DO NOT MESS THIS block
        //this block extracts the id and tries to pair the devices
        if (message.startsWith("MCU ID: ") || message.startsWith("Uid: ")) {
            String id = message.startsWith("MCU ID:")
                    ? message.substring(8).trim()
                    : message.substring(4).trim();
            idtosession.put(id, conn);
            String partnerID = null;
            if (message.startsWith("MCU ID:")) {
                partnerID = predefinedID.get(id);
            } else {
                for (Map.Entry<String, String> entry : predefinedID.entrySet()) {
                    if (entry.getValue().equals(id)) {
                        partnerID = entry.getKey();
                        break;
                    }
                }
            }
            if (partnerID != null && idtosession.containsKey(partnerID)) {
                WebSocket partnerSession = idtosession.get(partnerID);
                SessionToPartner.put(conn, partnerSession);
                SessionToPartner.put(partnerSession, conn);
                conn.send("Connected with " + partnerID);
                partnerSession.send("connected with " + id);
            } else {
                conn.send("Waiting for partner..");
            }
        }else{
            WebSocket partner = SessionToPartner.get(conn);
            if (partner != null && partner.isOpen()) {
                partner.send(message);
            }
        }
        

    }

    @Override
    public void onClose(WebSocket conn, int code, String reason, boolean remote) {
        String id = null;
        for (Map.Entry<String, WebSocket> entry : idtosession.entrySet()) {
            if (entry.getValue() == conn) {
                id = entry.getKey();
                break;
            }
        }
        if (id != null) {
            idtosession.remove(id);
            WebSocket partner = SessionToPartner.remove(conn);
            if (partner != null) {
                SessionToPartner.remove(partner);
                if (partner.isOpen()) {
                    partner.send("Partner disconnected.");
                }
            }
        }
        System.out.println("server connection of this client just ended.");
    }

    @Override
    public void onError(WebSocket conn, Exception ex) {
        System.err.println("an error Ocurred: " + ex);
    }

    public static void main(String[] args) {
        String myIp = NetworkUtils.Findip();

        String host = myIp;
        int port = 8081;

        mainServer server = new mainServer(new InetSocketAddress(host, port));
        System.out.println("The devices can now connect to the following address");
        if(myIp != null){
            System.out.println("ws://" + host + ":" + port);
        }
        server.start();
    }


}