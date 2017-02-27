#!/usr/bin/env python
import sys

def get_segs(f):
    segs = []
    cur_seg = []
    with open(f, 'r') as fw:
        for line in fw:
            line = line.strip()
            if line[0] == '@':
                line = line[1:].replace(' ','').replace('\n','')
                cur_seg = list()
                cur_seg.append(line)
                cur_seg.append(list())
                segs.append(cur_seg)
                print line
            elif line[0] == 'q':
                print "End"
            else:
                line = line[:].replace('\n','')
                line = line.split(' ')
                for each in line:
                    cur_seg[1].append(each)
    return segs
#len, da, ra, nr_high, nr_low, opcode = 1addr, 2, normal, , < payload >
packet_header = [0, 0, 0, 0, 0, 0]

packet_offset_len = 0
packet_offset_da = 1
packet_offset_ra = 2
packet_offset_nr_hi = 3
packet_offset_nr_lo = 4
packet_offset_opcode = 5
packet_offset_payload = 6

opcode_addr = '1'
opcode_data = '2'
addr_da = 'ad'
addr_ta = 'ca'
frag_idx = 0

preproc_preambles = '#pragma once\n'
n_pkt_string_start = 'uint16_t npkts[]= { \n'
n_pkt_string_end = '};\n'

pkt_def_start = 'uint8_t *packets[][#] = {' + '\n'
pkt_def_end = '};\n'
pkt_compound_literal_start = '(uint8_t []) { '
pkt_compound_literal_end = '},\n'

def generate_c_hdr(packet_lists):
    global pkt_def_start
    f = open('fw.h','w')
    f.write(preproc_preambles)
    npkts =  n_pkt_string_start
    for packet_list in packet_lists:
        npkts += ' ' + str(len(packet_list)) + ',' + '\n'
    npkts += n_pkt_string_end
    f.write(npkts)
    max_len = 0
    for each in packet_lists:
        if len(each) > max_len:
            max_len = len(each)
    pkt_def_start = pkt_def_start.replace('#', str(max_len))
    f.write(pkt_def_start)

    for packet_list in packet_lists:
        f.write('\t{\n');
        for each in packet_list:
            hex_prefixed = ['0x' + h for h in each]
            hex_prefixed = ', '.join(hex_prefixed)
            hex_prefixed = '\t'*2 + pkt_compound_literal_start + hex_prefixed + pkt_compound_literal_end
            f.write(hex_prefixed)
        f.write('\t},\n');
    f.write(pkt_def_end)

def prepare_fragments(payload, sizelimit):
    global frag_idx
    fragments = list()
    assert payload
    i = 0
    for i in range((len(payload) + sizelimit - 1)/sizelimit):
        fragment_payload = payload[i*sizelimit: (i+1)*sizelimit + 1]
        frag = packet_header + fragment_payload
        if i:
            frag[packet_offset_opcode] = (opcode_data)
        else:
            frag[packet_offset_opcode] = (opcode_addr)
        frag[packet_offset_da] = str(addr_da)
        frag[packet_offset_ra] = str(addr_ta)
        frag[packet_offset_nr_lo] = str(hex(frag_idx&0xff))[2:]
        frag[packet_offset_nr_hi] = str(hex((frag_idx>>8)&0xff))[2:]
        if not frag[packet_offset_nr_hi]:
             frag[packet_offset_nr_hi] = '0'
        frag_idx +=1 
        frag[packet_offset_len] = str(hex(len(frag)-1))[2:]
        fragments.append(frag)
    return fragments

def prepare_packets(segs, sizelimit = 54):
    global frag_idx
    frag_idx = 0
    packetlist = []
    for seg in segs:
        payload = []
        addr = seg[0]
        payload.append(addr[0:2])
        payload.append(addr[2:4])
        for byte in seg[1]: 
            payload.append(byte)
        seg_frags = prepare_fragments(payload, sizelimit)
        for each in seg_frags:
            packetlist.append(each)
    return packetlist   

"""
        data=fw.read().replace('\n', '')
        print(data)
"""

if __name__ == '__main__':
    packet_lists = []
    if len(sys.argv) < 2:
        print("Specify ti hex file")
    else:
        for f in sys.argv[1:]:
            segs =  get_segs(f)
            packet_lists.append(prepare_packets(segs, 54))
        generate_c_hdr(packet_lists)
