package com.ecosense.Utils;

import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.util.Enumeration;

public class NetworkUtils {

    public static String Findip() {
        try {
            Enumeration<NetworkInterface> NetInt = NetworkInterface.getNetworkInterfaces();
            while (NetInt.hasMoreElements()) {
                NetworkInterface iface = NetInt.nextElement();

                if (iface.isLoopback() || !iface.isUp()) {
                    continue;
                }

                Enumeration<InetAddress> address = iface.getInetAddresses();

                while (address.hasMoreElements()) {
                    InetAddress addr = address.nextElement();
                    if (addr instanceof Inet4Address) {
                        return addr.getHostAddress();

                    }
                }

            }
        } catch (Exception e) {
            System.err.println("Could not resolve the IP address of this system. "+e.getMessage());
        }
        return null;
    }
}
