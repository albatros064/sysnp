#ifndef SYSNP_TLB_H
#define SYSNP_TLB_H

namespace sysnp {

namespace nbus {

namespace n16r {

enum TlbMode {
    TlbRead,
    TlbWrite,
    TlbExecute
};

enum TlbResponse {
    TlbHit,
    TlbMiss,
    TlbNoExecute,
    TlbReadOnly,
    TlbKernelOnly,
    TlbNotPresent
};

class TlbEntry {
    public:
        TlbEntry();
        TlbEntry(uint32_t, uint32_t, bool, bool, bool, bool);
        virtual ~TlbEntry() {}

        TlbResponse translate(uint32_t, uint32_t&, bool, TlbMode);

        uint32_t getTag() { return tag; }
    private:
        bool kernelOnly;
        bool readOnly;
        bool dataOnly;
        bool present;

        bool occupied;

        uint32_t tag;
        uint32_t physical;
};
class TlbBin {
    public:
        TlbBin();
        virtual ~TlbBin() {}

        TlbResponse translate(uint32_t, uint32_t&, bool, TlbMode);
        void load(TlbEntry);
    private:
        std::vector<TlbEntry> ways;
        std::vector<uint8_t> lru;
};
class Tlb {
    public:
        Tlb();
        virtual ~Tlb() {};

        TlbResponse translate(uint32_t, uint32_t&, bool, TlbMode);
        void invalidate();
        void load(TlbEntry);
    private:
        std::vector<TlbBin> bins;
};

}; // namespace n16r

}; // namespace nbus

}; // namespace sysnp

#endif
