#pragma once

#include "global.cpp"
#include "common.cpp"
#include <map>
#include <memory>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/BooleanTopic.h>
#include <string>
#include <string_view>
#include <vector>

namespace Network {
namespace TagLocation {
    class Publisher {
        public:
        Publisher() = default;
        virtual ~Publisher() = default;

        virtual cobalt::promise<void> publish(std::vector<World::Pos2DwTag> tagLocations) = 0;
    };

    class NTPublisher : Publisher {
        public:
        NTPublisher() {
            m_tagsTable = nt::NetworkTableInstance::GetDefault().GetTable("Vision")->GetSubTable("Tags");
        }

        cobalt::promise<void> publish(std::vector<World::Pos2DwTag> tagLocations) {
            for (auto& tag : tagLocations) {
                std::shared_ptr<nt::NetworkTable> tagTab; 
                if (m_tagTabCache.contains(tag.id)) {
                    tagTab = m_tagTabCache[tag.id];
                } else {
                    tagTab = m_tagsTable->GetSubTable(std::to_string(tag.id));
                    m_tagTabCache.emplace(tag.id, tagTab);
                }

                tagTab->PutNumber("x", tag.x);
                tagTab->PutNumber("y", tag.y);
                tagTab->PutNumber("r", tag.rot);
            };
            co_return;
        }
        private:
        std::map<int, std::shared_ptr<nt::NetworkTable>> m_tagTabCache; 
        std::shared_ptr<nt::NetworkTable> m_tagsTable;
    };
}
};