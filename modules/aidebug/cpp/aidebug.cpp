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
    // Entity Pool
    std::vector<CBaseEntity*> m_pool;

    void OnInit() override
    {
        TracyZoneScoped;
        ShowInfo("AIDebug: Module Loaded");
    }

    void OnZoneTick(CZone* PZone) override
    {
        TracyZoneScoped;

        auto mobList = PZone->GetZoneEntities()->GetMobList();
        for (auto const& [targid, entity] : mobList)
        {
            auto mob = dynamic_cast<CMobEntity*>(entity);
            if (entity->GetLocalVar("AIDEBUG_ENABLED") == 1)
            {
                if (m_pool.empty())
                {
                    BuildDebugPool(PZone);
                }
                RenderPathForMob(PZone, mob);
            }
        }
    }

    void ClearRenderedMobs(CZone* PZone)
    {
        if (m_pool.empty())
        {
            return;
        }

        for (auto i = 0; i < m_pool.size(); i++)
        {
            if (m_pool[i]->status != STATUS_TYPE::DISAPPEAR)
            {
                m_pool[i]->FadeOut();
            }
        }
    }

    void RenderPathForMob(CZone* PZone, CMobEntity* PMob)
    {
        auto validPlayers = std::vector<CBaseEntity*>();
        auto players      = PZone->GetZoneEntities()->GetCharList();
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

        for (uint16 i = 0; i < rendered.size(); i++)
        {
            auto entity = m_pool[i];
            auto point  = rendered[i];

            entity->loc.p.x        = point.position.x;
            entity->loc.p.y        = point.position.y;
            entity->loc.p.z        = point.position.z;
            entity->loc.p.rotation = point.position.rotation;
            entity->status         = STATUS_TYPE::NORMAL;
            entity->updatemask |= UPDATE_STATUS;
            entity->updatemask |= UPDATE_POS;
        }
    }

    void BuildDebugPool(CZone* PZone)
    {
        // Initialize 128 Dynamic Entities in the list to use for this zone
        for (auto i = 0; i < 128; i++)
        {
            m_pool.push_back(BuildNPC(PZone));
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
        // PNpc->SetModelId(1507); // Runic Lamp
        PNpc->SetModelId(302); // Mandragora
        PZone->InsertNPC(PNpc);

        return PEntity;
    }
};

REGISTER_CPP_MODULE(AIDebugModule);
