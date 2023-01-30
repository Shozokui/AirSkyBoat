#include "ai/ai_container.h"
#include "common/mmo.h"
#include "entities/mobentity.h"
#include "entities/npcentity.h"
#include "mob_modifier.h"
#include "utils/mobutils.h"
#include "utils/moduleutils.h"
#include "zone.h"
#include "zone_entities.h"

#include <vector>

class AIDebugModule : public CPPModule
{
    std::map<uint16, std::vector<CBaseEntity*>> m_entities;
    std::map<uint16, uint16>                    m_used;

    void OnInit() override
    {
        TracyZoneScoped;
        ShowInfo("AIDebug: Module Loaded");
    }

    void OnZoneTick(CZone* PZone) override
    {
        TracyZoneScoped;

        ClearRenderedMobs(PZone);

        auto mobList = PZone->GetZoneEntities()->GetMobList();
        for (auto const& [targid, entity] : mobList)
        {
            auto mob = dynamic_cast<CMobEntity*>(entity);
            if (entity->GetLocalVar("AIDEBUG_ENABLED") == 1)
            {
                if (m_entities.size() < 1)
                {
                    BuildDebugPool(PZone);
                }
                RenderPathForMob(PZone, mob);
            }
        }
    }

    void ClearRenderedMobs(CZone* PZone)
    {
        if (m_entities.find(PZone->GetID()) != m_entities.end() && m_entities[PZone->GetID()].size() < 1)
        {
            return;
        }

        for (auto i = m_used[PZone->GetID()]; i < m_entities[PZone->GetID()].size(); i++)
        {
            if (m_entities[PZone->GetID()][i]->status != STATUS_TYPE::DISAPPEAR)
            {
                m_entities[PZone->GetID()][i]->FadeOut();
            }
        }
    }

    void RenderPathForMob(CZone* PZone, CMobEntity* PMob)
    {
        auto validPlayers = std::vector<CBaseEntity*>();

        auto players = PZone->GetZoneEntities()->GetCharList();
        for (auto const& [id, player] : players)
        {
            if (player->GetLocalVar("AIDEBUG_ENABLED") == 1)
            {
                validPlayers.push_back(player);
            }
        }

        if (validPlayers.empty())
        {
            return;
        }

        auto player   = validPlayers[0];
        auto path     = PMob->PAI->PathFind->GetPath();
        auto rendered = std::vector<pathpoint_t>();

        for (uint16 i = 0; i < path.size(); i++)
        {
            auto point = path[i];
            if (distance(player->loc.p, point.position) < 50)
            {
                rendered.push_back(point);
            }
        }

        m_used[PZone->GetID()] = static_cast<uint16>(rendered.size());
        for (uint16 i = 0; i < rendered.size(); i++)
        {
            m_entities[PZone->GetID()][i]->loc.p.x        = rendered[i].position.x;
            m_entities[PZone->GetID()][i]->loc.p.y        = rendered[i].position.y;
            m_entities[PZone->GetID()][i]->loc.p.z        = rendered[i].position.z;
            m_entities[PZone->GetID()][i]->loc.p.rotation = rendered[i].position.rotation;
            m_entities[PZone->GetID()][i]->loc.p.moving   = 0;
            m_entities[PZone->GetID()][i]->status         = STATUS_TYPE::NORMAL;
            m_entities[PZone->GetID()][i]->updatemask |= UPDATE_STATUS;
            m_entities[PZone->GetID()][i]->updatemask |= UPDATE_POS;
        }
    }

    void BuildDebugPool(CZone* PZone)
    {
        if (m_entities[PZone->GetID()].size() > 0)
        {
            return;
        }

        for (auto i = 0; i < 128; i++)
        {
            m_entities[PZone->GetID()].push_back(BuildNPC(PZone));
        }
    }

    CBaseEntity* BuildNPC(CZone* PZone)
    {
        auto PEntity     = new CNpcEntity();
        PEntity->name    = "NavPoint";
        PEntity->objtype = TYPE_NPC;
        PZone->GetZoneEntities()->AssignDynamicTargIDandLongID(PEntity);
        PEntity->name      = "NavPoint";
        PEntity->isRenamed = true;

        auto PNpc     = dynamic_cast<CNpcEntity*>(PEntity);
        PNpc->namevis = NAMEVIS::VIS_NONE;
        PNpc->SetUntargetable(true);
        PNpc->m_triggerable = false;
        PNpc->status        = STATUS_TYPE::NORMAL;
        PNpc->SetModelId(302); // Mandragora
        PZone->InsertNPC(PNpc);

        return PEntity;
    }
};

REGISTER_CPP_MODULE(AIDebugModule);
