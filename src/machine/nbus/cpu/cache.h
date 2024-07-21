#ifndef SYSNP_CACHE_H
#define SYSNP_CACHE_H

#include <cstdint>
#include <vector>
#include <type_traits>

namespace sysnp {

namespace nbus {

namespace n16r {

enum CacheCheck {
    CacheContainsNone,
    CacheContainsSingle,
    CacheContainsSplit,
    CacheContainsPartial
};

template<typename A, typename V, typename M, typename F>
    //requires
        //std::is_integral<A>::value && !std::is_signed<A>::value &&
        //std::is_integral<F>::value && !std::is_signed<F>::value
class Cache {
    public:
        Cache() {}
        Cache(int _binBits, int _lineBits, int _wayBits, bool _singleValue, int _dirtyFlag, int _presentFlag):
                binBits(_binBits), lineBits(_lineBits), wayBits(_wayBits), singleValue(singleValue),
                dirtyFlag(_dirtyFlag), presentFlag(_presentFlag),
                tagMask(0), binMask(0), lineMask(0) {
            contentCount = _singleValue ? 1 : (1 << lineBits);

            wayCount = 1 << wayBits;
            int addressBits = sizeof(A) * 8;
            int tagBits = addressBits - binBits - lineBits;

            tagMask  = ~tagMask;
            binMask  = ~binMask;
            lineMask = ~lineMask;

            tagMask  = tagMask << (binBits + lineBits);
            binMask  = ((binMask << tagBits) >> (tagBits + lineBits)) << lineBits;
            lineMask = lineMask >> (addressBits - lineBits);

            int binCount = 1 << binBits;
            int entryCount = binCount * wayCount;

            tag     .resize(entryCount, 0);
            lru     .resize(entryCount, 0);
            meta    .resize(entryCount);
            flag    .resize(entryCount);
            content .resize(entryCount * contentCount);

            tag     .shrink_to_fit();
            meta    .shrink_to_fit();
            lru     .shrink_to_fit();
            flag    .shrink_to_fit();
            content .shrink_to_fit();
        }
        virtual ~Cache() {}

        CacheCheck contains(A address, int count, M metaValue) {
            A addressTag = address & tagMask;
            A minBin = address & binMask;
            A maxBin = (address + count - 1) & binMask;
            int binNumber = (minBin >> lineBits) << wayBits;

            bool startFound = false;
            for (int w = 0; w < wayCount; w++) {
                if (tag[binNumber + w] == addressTag && meta[binNumber + w] == metaValue) {
                    if (presentFlag >= 0 && (flag[binNumber + w] & (1 << presentFlag))) {
                        startFound = true;
                        break;
                    }
                }
            }

            if (minBin != maxBin) {
                binNumber = (maxBin >> lineBits) << wayBits;

                bool endFound = false;
                for (int w = 0; w < wayCount; w++) {
                    if (tag[binNumber + w] == addressTag && meta[binNumber + w] == metaValue) {
                        if (presentFlag >= 0 && (flag[binNumber + w] & (1 << presentFlag))) {
                            endFound = true;
                            break;
                        }
                    }
                }

                if (startFound && endFound) {
                    return CacheContainsSplit;
                }
                else if (startFound || endFound) {
                    return CacheContainsPartial;
                }
            }

            if (startFound) {
                return CacheContainsSingle;
            }
            return CacheContainsNone;
        }

        std::vector<V> get(A address, int count, M metaValue, bool updateLru = false) {
            std::vector<V> data(count);

            int lastBinNum = -1;
            int lineIndex = -1;
            for (int i = 0; i < count; i++) {
                A valueAddress = address + i;
                A addressTag = valueAddress & tagMask;
                int binNum = binNumber(valueAddress);
                if (binNum != lastBinNum) {
                    lastBinNum = binNum;
                    lineIndex = -1;
                    for (int w = 0; w < wayCount; w++) {
                        int offset = binNum + w;
                        if (tag[offset] == addressTag && meta[offset] == metaValue) {
                            if (presentFlag >= 0 && !(flag[offset] & (1 << presentFlag))) {
                                continue;
                            }
                            lineIndex = offset;
                            if (updateLru) {
                                int8_t oldLru = lru[offset];
                                for (int wl = 0; wl < wayCount; wl++) {
                                    if (wl == w) {
                                        lru[binNum + wl] = wayCount - 1;
                                    }
                                    else if (lru[binNum + wl] > oldLru) {
                                        lru[binNum + wl] -= 1;
                                    }
                                }
                            }
                            break;
                        }
                    }
                }

                if (lineIndex >= 0) {
                    A lineOffset = lineIndex;
                    if (!singleValue) {
                        lineOffset <<= lineBits;
                        lineOffset += valueAddress & lineMask;
                    }
                    data[i] = content[lineOffset];
                }
                else {
                    data[i] = 0;
                }
            }

            return data;
        }

        std::vector<V> read(A address, int count, M metaValue) {
            return get(address, count, metaValue, true);
        }

        void load(A address, M metaValue, F flags, std::vector<V> values) {
            int selectedLine = selectLine(address, metaValue);
            load(address, metaValue, flags, selectedLine, values);
        }
        void load(A address, M metaValue, F flags, int way, std::vector<V> values) {
            int binNum = binNumber(address);
            int lineNumber = binNum + way;
            A lineStartIndex = lineNumber;
            if (!singleValue) {
                lineStartIndex <<= lineBits;
            }
            for (int i = 0; i < contentCount && i < values.size(); i++) {
                content[lineStartIndex + i] = values[i];
            }

            tag  [lineNumber] = address & tagMask;
            meta [lineNumber] = metaValue;
            flag [lineNumber] = flags;

            auto oldLru = lru[lineNumber];
            lru[lineNumber] = (wayCount) - 1;

            if (presentFlag >= 0) {
                flag[lineNumber] |= (1 << presentFlag);
            }

            for (int i = 0; i < wayCount; i++) {
                if (i == way) {
                    continue;
                }
                if (lru[binNum + i] > oldLru) {
                    lru[binNum + i]--;
                }
            }
        }

        void write(A address, M metaValue, std::vector<V> values) {
            int lastBinNum = -1;
            int lineIndex = -1;
            for (int i = 0; i < values.size(); i++) {
                A valueAddress = address + i;
                A addressTag = valueAddress & tagMask;
                int binNum = binNumber(valueAddress);
                if (binNum != lastBinNum) {
                    lastBinNum = binNum;
                    if (lastBinNum >= 0 && lineIndex >= 0 && dirtyFlag >= 0) {
                        flag[lineIndex] &= ~(1 << dirtyFlag);
                    }
                    lineIndex = -1;
                    for (int w = 0; w < wayCount; w++) {
                        int offset = binNum + w;
                        if (presentFlag >= 0 && !(flag[offset] & (1 << presentFlag))) {
                            continue;
                        }
                        if (tag[offset] == addressTag && meta[offset] == metaValue) {
                            lineIndex = offset;

                            if (dirtyFlag >= 0) {
                                flag[lineIndex] |= (1 << dirtyFlag);
                            }

                            int8_t oldLru = lru[offset];
                            for (int wl = 0; wl < wayCount; wl++) {
                                if (wl == w) {
                                    lru[binNum + wl] = wayCount - 1;
                                }
                                else if (lru[binNum + wl] > oldLru) {
                                    lru[binNum + wl] -= 1;
                                }
                            }
                            break;
                        }
                    }
                }

                if (lineIndex >= 0) {
                    A lineOffset = lineIndex;
                    if (!singleValue) {
                        lineOffset <<= lineBits;
                        lineOffset += valueAddress & lineMask;
                    }
                    content[lineOffset] = values[i];
                }
            }
        }

        A getLineMask() { return lineMask; }
        int getLineBytes() { return contentCount; }

        int selectLine(A address, M metaValue) {
            int binNum = binNumber(address);

            if (presentFlag >= 0) {
                F flagMask = (1 << presentFlag);
                for (int i = 0; i < wayCount; i++) {
                    if (flag[binNum + i] & flagMask) {
                        return i;
                    }
                }
            }
            for (int i = 0; i < wayCount; i++) {
                if (lru[binNum + i] == 0) {
                    return i;
                }
            }
            return 0;
        }

    private:
        std::vector<A     > tag;
        std::vector<M     > meta;
        std::vector<V     > content;
        std::vector<F     > flag;
        std::vector<int8_t> lru;

        A tagMask;
        A binMask;
        A lineMask;

        int binBits;
        int lineBits;
        int wayBits;

        int wayCount;
        int contentCount;
        bool singleValue;
        int dirtyFlag;
        int presentFlag;

        int binNumber(A address) { return ((address & binMask) >> lineBits) << wayBits; }
};

}; // namespace n16r

}; // namespace nbus

}; // namespace sysnp

#endif
