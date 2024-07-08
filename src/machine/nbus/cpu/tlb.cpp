#include "tlb.h"

namespace sysnp {

namespace nbus {

namespace n16r {

Tlb::Tlb():bins(16) {}

TlbResponse Tlb::translate(uint32_t virtualAddress, uint32_t &physicalAddress, bool kernelMode, TlbMode accessType) {
    if (kernelMode) {
        uint32_t checkAddress = virtualAddress & 0xfff00000;
        if (checkAddress == 0x100000 || checkAddress == 0xf00000) {
            physicalAddress = virtualAddress;
            return TlbHit;
        }
    }

    int bin = (virtualAddress & 0xf000) >> 12;

    return bins[bin].translate(virtualAddress, physicalAddress, kernelMode, accessType);
}

void Tlb::invalidate() {
    for (int i = 0; i < bins.size(); i++) {
        TlbBin bin;
        bins[i] = bin;
    }
}

void Tlb::load(uint16_t , TlbEntry entry) {
    int bin = (entry& 0xf000) >> 12;
    bins[bin].load(entry);
}

TlbBin::TlbBin():ways(4), lru(4, 0), {}

TlbResponse TlbBin::translate(uint32_t virtualAddress, uint32_t &physicalAddress, bool kernelMode, TlbMode accessType) {
    for (int i = 0; i < ways.length; i++) {
        auto response = ways[i].translate(virtualAddress, physicalAddress, kernelMode, accessType);
        if (response != TlbMiss) {
            auto current = lru[i];
            lru[i] = 3;

            for (int l = 0; l < lru.length; l++) {
                if (l == i) {
                    lru[l] = 3;
                }
                else if (lru[l] >= current) {
                    lru[l] = lru[l] - 1;
                }
            }
            return response;
        }
    }

    return TlbResponse::TlbMiss;
}

void TlbBin::load(TlbEntry entry) {
    for (int i = 0; i < ways.length; i++) {
        if (lru[i] == 0) {
            ways[i] = entry;
        }
    }
}

TlbEntry::TlbEntry():occupied(false) {}
TlbEntry::TlbEntry(uint32_t _tag, uint32_t _physical, bool _kernel, bool _readOnly, bool _noExecute, bool _present):
    tag(_tag), physical(_physical),
    kernelOnly(_kernel), readOnly(_readOnly), dataOnly(_noExecute),
    present(_present), occupied(true) {}

TlbResponse TlbEntry::translate(uint32_t virtualAddress, uint32_t &physicalAddress, bool kernelMode, TlbMode accessType) {
    if (occupied && (virtualAddress & 0xfffff000) == tag) {
        if (kernelOnly && !kernelMode) {
            return TlbKernelOnly;
        }
        if (dataOnly && accessType == TlbExecute) {
            return TlbNoExecute;
        }
        if (readOnly && accessType == TlbWrite) {
            return TlbReadOnly;
        }
        if (!present) {
            return TlbNotPresent;
        }

        physicalAddress = (virtualAddress & 0xffff) | physical;

        return TlbHit;
    }
    return TlbMiss;
}

}; // namespace n16r

}; // namespace nbus

}; // namespace sysnp

