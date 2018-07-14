import sys

class MessageAssertion(object):
    def __init__(self, message, test_case):
        self.message = message
        self.test_case = test_case

    def is_mle(self):
        self.test_case.assertEqual(self.message.type, MessageType.MLE, 'Invalid message type. Expected MLE message.')
        return self

    def mle_is_type(self, command_type):
        self.test_case.assertEqual(self.message.mle.command.type, command_type)
        return self

    def mle_with_tlv(self, tlv_class_type, optional=False):
        """To confirm if Mle message contains the TLV type.

        Args:
            tlv_class_type: tlv's type.

        Returns:
            mle.Route64: If contains the TLV, return it.
        """
        contains_tlv = False
        for tlv in self.mle.command.tlvs:
            if isinstance(tlv, tlv_class_type):
                contains_tlv = True
                break

        if not optional:
            self.test_case.assertTrue(contains_tlv)
        else:
            if contains_tlv == True:
                print("MleMessage contains optional TLV: {}".format(tlv_class_type))
            else:
                print("MleMessage doesn't contain optional TLV: {}".format(tlv_class_type))

        return self

    def mle_without_tlv(self, tlv_class_type):
        contains_tlv = False
        for tlv in self.message.mle.command.tlvs:
            if isinstance(tlv, tlv_class_type):
                contains_tlv = True
                break

        self.assertFalse(contains_tlv)

        return self

    def mle_with_router_quantity(self, router_quantity):
        """Confirm if Leader contains the Route64 TLV with router_quantity assigned Router IDs.

        Args:
            router_quantity: the quantity of router.
        """
        tlv = self.message.get_mle_message_tlv(mle.Route64)
        self.test_case.assertNotEqual(tlv, None)
        router_id_mask = tlv.router_id_mask

        count = 0
        for i in range(1, 65):
            count += (router_id_mask & 1)
            router_id_mask = (router_id_mask >> 1)
        self.test_case.assertEqual(count, router_quantity)

        return self

    def is_coap(self):
        self.test_case.assertEqual(self.message.type, MessageType.COAP, 'Invalid message type. Expected CoAP message.')
        return self

    def coap_with_tlv(self, tlv_class_type, optional=False):

        contains_tlv = False
        for tlv in self.message.coap.payload:
            if isinstance(tlv, tlv_class_type):
                contains_tlv = True
                break

        if not optional:
            self.test_case.assertTrue(contains_tlv)
        else:
            if contains_tlv:
                print("CoapMessage contains optional TLV: {}".format(tlv_class_type))
            else:
                print("CoapMessage doesn't contain optional TLV: {}".format(tlv_class_type))

        return self

    def coap_without_tlv(self, tlv_class_type):
        contains_tlv = False
        for tlv in self.coap.payload:
            if isinstance(tlv, tlv_class_type):
                contains_tlv = True
                break

        self.test_case.assertFalse(contains_tlv)

        return self

    def coap_with_uri_path(self, uri_path):
        self.test_case.assertEqual(uri_path, self.message.coap.uri_path)
        return self

    def coap_with_code(self, code):
        self.test_case.assertEqual(code, self.coap.code)
        return self

    def is_sent_to_node(self, node):
        sent_to_node = False
        dst_addr = self.message.ipv6_packet.ipv6_header.destination_address

        for addr in node.get_addrs():
            if dst_addr == ipaddress.ip_address(addr):
                sent_to_node = True

        if self.message.mac_header.dest_address.type == common.MacAddressType.SHORT:
            mac_address = common.MacAddress.from_rloc16(node.get_addr16())
            if self.message.mac_header.dest_address == mac_address:
                sent_to_node = True

        elif self.message.mac_header.dest_address.type == common.MacAddressType.LONG:
            mac_address = common.MacAddress.from_eui64(bytearray(node.get_addr64(), encoding="utf-8"))
            if self.message.mac_header.dest_address == mac_address:
                sent_to_node = True

        self.test_case.assertTrue(sent_to_node)

        return self

    def is_sent_to_addr(self, ipv6_address):
        if sys.version_info[0] == 2:
            ipv6_address = ipv6_address.decode("utf-8")

        self.test_case.assertEqual(self.ipv6_packet.ipv6_header.destination_address, ipaddress.ip_address(ipv6_address))

        return self

    def with_hop_limit(self, hop_limit):
        self.test_case.assertEqual(self.ipv6_packet.ipv6_header.hop_limit, hop_limit)
